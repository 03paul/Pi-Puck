import json
import math
import threading
import time

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck


CLIENT_VERSION = "no-circle-forward-only-v3"


# =========================
# CONFIG
# =========================
BROKER = "192.168.178.43"
PORT = 1883
TOPIC = "robot_pos/all"
MY_ID = "39"

ARENA_X_MIN = 0.0
ARENA_X_MAX = 2.0
ARENA_Y_MIN = 0.0
ARENA_Y_MAX = 1.0

# Safety
MQTT_TIMEOUT_S = 0.75
EDGE_MARGIN_M = 0.08

# Target behavior
CATCH_DISTANCE_M = 0.12
CATCH_RELEASE_DISTANCE_M = 0.16
FAST_DISTANCE_M = 0.70
MIN_APPROACH_SPEED = 250
MAX_SPEED = 1000

# Steering behavior
LOOP_PERIOD_S = 0.05
AIM_OK_DEG = 6.0
STEER_GAIN = 0.85
SHARP_TURN_DEG = 70.0
SHARP_TURN_SPEED = 220
MIN_MOVING_SPEED = 120
ANGLE_UNIT = "auto"  # "auto", "degrees", or "radians"
ANGLE_SIGN = 1       # set to -1 if the MQTT angle grows in the wrong direction
ANGLE_OFFSET_DEG = 0 # try 90, -90, or 180 if the tracked front is rotated
USE_MOTION_HEADING = True
MOTION_HEADING_MIN_STEP_M = 0.015

# Set this to -1 if the robot steers away from the target instead of toward it.
STEERING_SIGN = 1


# =========================
# SHARED MQTT STATE
# =========================
positions_lock = threading.Lock()
robot_positions = {}
last_message_time = 0.0


# =========================
# MATH / DATA HELPERS
# =========================
def clamp(value, low, high):
    return max(low, min(high, value))


def angle_diff_deg(target_deg, current_deg):
    """Shortest signed difference target-current in degrees, range [-180, 180)."""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def normalize_angle_deg(raw_angle):
    angle = float(raw_angle)

    if ANGLE_UNIT == "radians":
        angle = math.degrees(angle)
    if ANGLE_UNIT == "degrees":
        angle = angle
    elif -2.0 * math.pi <= angle <= 2.0 * math.pi:
        angle = math.degrees(angle)

    return (ANGLE_SIGN * angle + ANGLE_OFFSET_DEG) % 360.0


def parse_robot_state(raw):
    if not isinstance(raw, dict):
        return None

    pos = raw.get("position")
    angle = raw.get("angle")
    if not isinstance(pos, (list, tuple)) or len(pos) < 2 or angle is None:
        return None

    try:
        return {
            "x": float(pos[0]),
            "y": float(pos[1]),
            "angle": normalize_angle_deg(angle),
        }
    except (TypeError, ValueError):
        return None


def get_snapshot():
    with positions_lock:
        return dict(robot_positions), last_message_time


def get_my_state(positions):
    return parse_robot_state(positions.get(MY_ID))


def distance_and_angle(from_state, to_state):
    dx = to_state["x"] - from_state["x"]
    dy = to_state["y"] - from_state["y"]
    distance = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx)) % 360.0
    return distance, angle


def find_closest_robot(my_state, positions):
    closest_id = None
    closest_state = None
    closest_distance = float("inf")

    for robot_id, raw_state in positions.items():
        if str(robot_id) == MY_ID:
            continue

        other_state = parse_robot_state(raw_state)
        if other_state is None:
            continue

        distance, _ = distance_and_angle(my_state, other_state)
        if distance < closest_distance:
            closest_id = str(robot_id)
            closest_state = other_state
            closest_distance = distance

    return closest_id, closest_state, closest_distance


def too_close_to_edge(state):
    return (
        state["x"] <= ARENA_X_MIN + EDGE_MARGIN_M
        or state["x"] >= ARENA_X_MAX - EDGE_MARGIN_M
        or state["y"] <= ARENA_Y_MIN + EDGE_MARGIN_M
        or state["y"] >= ARENA_Y_MAX - EDGE_MARGIN_M
    )


def speed_for_distance(distance_m):
    if distance_m <= CATCH_DISTANCE_M:
        return 0
    if distance_m >= FAST_DISTANCE_M:
        return MAX_SPEED

    span = FAST_DISTANCE_M - CATCH_DISTANCE_M
    factor = (distance_m - CATCH_DISTANCE_M) / span
    speed = MIN_APPROACH_SPEED + factor * (MAX_SPEED - MIN_APPROACH_SPEED)
    return int(clamp(speed, MIN_APPROACH_SPEED, MAX_SPEED))


# =========================
# ROBOT CONTROL
# =========================
pipuck = PiPuck(epuck_version=2)


def set_motors(left, right):
    left = int(clamp(left, -MAX_SPEED, MAX_SPEED))
    right = int(clamp(right, -MAX_SPEED, MAX_SPEED))
    pipuck.epuck.set_motor_speeds(left, right)


def stop_motors():
    set_motors(0, 0)


def drive_toward_target(my_state, target_state, distance_m, steering_sign):
    _, target_angle = distance_and_angle(my_state, target_state)
    diff = angle_diff_deg(target_angle, my_state["angle"])
    signed_diff = steering_sign * diff

    speed = speed_for_distance(distance_m)
    if abs(diff) > SHARP_TURN_DEG:
        speed = min(speed, SHARP_TURN_SPEED)

    if abs(diff) <= AIM_OK_DEG:
        left = speed
        right = speed
    elif signed_diff > 0:
        turn_ratio = clamp(abs(diff) / 100.0 * STEER_GAIN, 0.15, 0.80)
        left = speed * (1.0 - turn_ratio)
        right = speed
    else:
        turn_ratio = clamp(abs(diff) / 100.0 * STEER_GAIN, 0.15, 0.80)
        left = speed
        right = speed * (1.0 - turn_ratio)

    if speed > 0:
        left = clamp(left, MIN_MOVING_SPEED, MAX_SPEED)
        right = clamp(right, MIN_MOVING_SPEED, MAX_SPEED)
    else:
        left = 0
        right = 0

    set_motors(left, right)
    return target_angle, diff, speed, int(left), int(right)


# =========================
# MQTT
# =========================
def on_connect(client, userdata, flags, reason_code, properties=None):
    try:
        success = int(reason_code) == 0
    except (TypeError, ValueError):
        success = str(reason_code).lower() == "success"

    if success:
        client.subscribe(TOPIC)
        print(f"MQTT connected, subscribed to {TOPIC}")
    else:
        print(f"MQTT connect failed: {reason_code}")


def on_disconnect(client, userdata, *args):
    print("MQTT disconnected")
    stop_motors()


def on_message(client, userdata, msg):
    global robot_positions, last_message_time

    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
        if not isinstance(data, dict):
            return
    except (UnicodeDecodeError, json.JSONDecodeError):
        return

    with positions_lock:
        robot_positions = data
        last_message_time = time.monotonic()


def create_mqtt_client():
    client_id = f"pipuck_chaser_{MY_ID}"

    if hasattr(mqtt, "CallbackAPIVersion"):
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
        )
    else:
        client = mqtt.Client(client_id=client_id)

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    return client


# =========================
# MAIN LOOP
# =========================
def main():
    client = create_mqtt_client()
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    last_status = None
    last_debug_time = 0.0
    caught_robot_id = None
    steering_sign = STEERING_SIGN
    last_turn_check_time = 0.0
    last_turn_error = None
    last_target_id = None
    last_my_position = None
    motion_heading = None

    def status(text):
        nonlocal last_status
        if text != last_status:
            print(text)
            last_status = text

    try:
        print(f"PiPuck {MY_ID} started - {CLIENT_VERSION}")

        while True:
            now = time.monotonic()
            positions, message_time = get_snapshot()
            data_age = now - message_time if message_time > 0 else float("inf")

            if data_age > MQTT_TIMEOUT_S:
                stop_motors()
                status("Stopping: no fresh MQTT data")
                time.sleep(LOOP_PERIOD_S)
                continue

            my_state = get_my_state(positions)
            if my_state is None:
                stop_motors()
                status(f"Stopping: own robot {MY_ID} missing in MQTT data")
                time.sleep(LOOP_PERIOD_S)
                continue

            if last_my_position is not None:
                dx = my_state["x"] - last_my_position[0]
                dy = my_state["y"] - last_my_position[1]
                moved = math.hypot(dx, dy)
                if moved >= MOTION_HEADING_MIN_STEP_M:
                    motion_heading = math.degrees(math.atan2(dy, dx)) % 360.0
            last_my_position = (my_state["x"], my_state["y"])

            control_state = dict(my_state)
            heading_source = "mqtt"
            if USE_MOTION_HEADING and motion_heading is not None:
                control_state["angle"] = motion_heading
                heading_source = "motion"

            if too_close_to_edge(my_state):
                stop_motors()
                status(
                    "Stopping: too close to edge "
                    f"(x={my_state['x']:.3f}, y={my_state['y']:.3f})"
                )
                time.sleep(LOOP_PERIOD_S)
                continue

            target_id, target_state, distance_m = find_closest_robot(
                my_state,
                positions,
            )
            if target_id is None:
                stop_motors()
                status("Stopping: no other robot found")
                time.sleep(LOOP_PERIOD_S)
                continue

            if target_id != last_target_id:
                last_target_id = target_id
                last_turn_error = None
                last_turn_check_time = now

            if distance_m <= CATCH_DISTANCE_M:
                stop_motors()
                if caught_robot_id != target_id:
                    print(f"CAUGHT ROBOT {target_id}")
                    caught_robot_id = target_id
                status(f"Target {target_id} caught at {distance_m:.3f} m")
                time.sleep(LOOP_PERIOD_S)
                continue

            if caught_robot_id == target_id and distance_m > CATCH_RELEASE_DISTANCE_M:
                caught_robot_id = None

            target_angle, diff, speed, left_speed, right_speed = drive_toward_target(
                control_state,
                target_state,
                distance_m,
                steering_sign,
            )
            status(f"Chasing robot {target_id}")

            abs_diff = abs(diff)
            if abs_diff > SHARP_TURN_DEG and now - last_turn_check_time >= 1.0:
                if last_turn_error is not None and abs_diff > last_turn_error + 12.0:
                    steering_sign *= -1
                    print(f"Steering direction changed to {steering_sign}")
                    last_turn_error = None
                else:
                    last_turn_error = abs_diff
                last_turn_check_time = now

            if now - last_debug_time >= 0.5:
                print(
                    f"target={target_id} dist={distance_m:.3f}m "
                    f"heading={control_state['angle']:.1f}deg/{heading_source} "
                    f"target_angle={target_angle:.1f}deg "
                    f"diff={diff:.1f}deg speed={speed} "
                    f"motors=({left_speed},{right_speed}) "
                    f"steering_sign={steering_sign}"
                )
                last_debug_time = now

            time.sleep(LOOP_PERIOD_S)

    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping robot")
    finally:
        stop_motors()
        client.loop_stop()
        try:
            client.disconnect()
        except Exception:
            pass
        print("Stopped")


if __name__ == "__main__":
    main()
