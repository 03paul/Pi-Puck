# CODEX UPDATED THIS FILE - MOTION HEADING NO-SPIN VERSION
import json
import math
import threading
import time

import paho.mqtt.client as mqtt
from pipuck.pipuck import PiPuck


CLIENT_VERSION = "motion-heading-no-spin-v4"

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
MIN_APPROACH_SPEED = 240
MAX_SPEED = 1000

# Motion-heading control
LOOP_PERIOD_S = 0.05
BOOTSTRAP_SPEED = 220
BOOTSTRAP_MOVE_M = 0.025
HEADING_MIN_STEP_M = 0.012
STRAIGHT_TOLERANCE_DEG = 12.0
SHARP_TURN_DEG = 65.0
TURN_SPEED = 240
INNER_WHEEL_RATIO = 0.45
MIN_WHEEL_SPEED = 110

# Flip this to -1 if the robot consistently curves away from the target.
STEERING_SIGN = 1

# If one motor is mounted/reported reversed, change its sign to -1.
LEFT_MOTOR_SIGN = 1
RIGHT_MOTOR_SIGN = 1


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
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def parse_robot_state(raw):
    if not isinstance(raw, dict):
        return None

    pos = raw.get("position")
    angle = raw.get("angle", 0.0)
    if not isinstance(pos, (list, tuple)) or len(pos) < 2:
        return None

    try:
        return {
            "x": float(pos[0]),
            "y": float(pos[1]),
            "angle": float(angle) % 360.0,
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
    left = int(clamp(left, 0, MAX_SPEED)) * LEFT_MOTOR_SIGN
    right = int(clamp(right, 0, MAX_SPEED)) * RIGHT_MOTOR_SIGN
    pipuck.epuck.set_motor_speeds(left, right)


def stop_motors():
    pipuck.epuck.set_motor_speeds(0, 0)


def drive_straight(speed):
    speed = int(clamp(speed, 0, MAX_SPEED))
    set_motors(speed, speed)
    return speed, speed


def drive_toward_heading(heading_deg, target_state, my_state, steering_sign):
    distance_m, target_angle = distance_and_angle(my_state, target_state)
    diff = angle_diff_deg(target_angle, heading_deg)
    signed_diff = steering_sign * diff

    if abs(diff) <= STRAIGHT_TOLERANCE_DEG:
        speed = speed_for_distance(distance_m)
        left, right = drive_straight(speed)
        return target_angle, diff, speed, left, right

    speed = speed_for_distance(distance_m)
    if abs(diff) >= SHARP_TURN_DEG:
        speed = min(speed, TURN_SPEED)

    outer = max(speed, MIN_WHEEL_SPEED)
    inner = max(int(outer * INNER_WHEEL_RATIO), MIN_WHEEL_SPEED)

    if signed_diff > 0:
        left, right = inner, outer
    else:
        left, right = outer, inner

    set_motors(left, right)
    return target_angle, diff, speed, left, right


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
    motion_heading = None
    last_position = None
    bootstrap_start_position = None
    steering_sign = STEERING_SIGN
    last_progress_time = 0.0
    last_progress_distance = None

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

            if too_close_to_edge(my_state):
                stop_motors()
                status(
                    "Stopping: too close to edge "
                    f"(x={my_state['x']:.3f}, y={my_state['y']:.3f})"
                )
                time.sleep(LOOP_PERIOD_S)
                continue

            if last_position is not None:
                dx = my_state["x"] - last_position[0]
                dy = my_state["y"] - last_position[1]
                moved = math.hypot(dx, dy)
                if moved >= HEADING_MIN_STEP_M:
                    motion_heading = math.degrees(math.atan2(dy, dx)) % 360.0
            last_position = (my_state["x"], my_state["y"])

            target_id, target_state, distance_m = find_closest_robot(
                my_state,
                positions,
            )
            if target_id is None:
                stop_motors()
                status("Stopping: no other robot found")
                time.sleep(LOOP_PERIOD_S)
                continue

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

            if motion_heading is None:
                if bootstrap_start_position is None:
                    bootstrap_start_position = (my_state["x"], my_state["y"])

                moved_from_start = math.hypot(
                    my_state["x"] - bootstrap_start_position[0],
                    my_state["y"] - bootstrap_start_position[1],
                )
                if moved_from_start < BOOTSTRAP_MOVE_M:
                    left, right = drive_straight(BOOTSTRAP_SPEED)
                    status("Calibrating motion heading: driving straight")
                    if now - last_debug_time >= 0.5:
                        print(
                            f"bootstrap moved={moved_from_start:.3f}m "
                            f"motors=({left},{right})"
                        )
                        last_debug_time = now
                    time.sleep(LOOP_PERIOD_S)
                    continue
                motion_heading = math.degrees(
                    math.atan2(
                        my_state["y"] - bootstrap_start_position[1],
                        my_state["x"] - bootstrap_start_position[0],
                    )
                ) % 360.0

            target_angle, diff, speed, left_speed, right_speed = drive_toward_heading(
                motion_heading,
                target_state,
                my_state,
                steering_sign,
            )
            status(f"Chasing robot {target_id}")

            if now - last_progress_time >= 1.0:
                if (
                    last_progress_distance is not None
                    and distance_m > last_progress_distance + 0.03
                    and abs(diff) > STRAIGHT_TOLERANCE_DEG
                ):
                    steering_sign *= -1
                    print(f"Steering direction flipped to {steering_sign}")
                last_progress_distance = distance_m
                last_progress_time = now

            if now - last_debug_time >= 0.5:
                print(
                    f"target={target_id} dist={distance_m:.3f}m "
                    f"heading={motion_heading:.1f}deg "
                    f"target_angle={target_angle:.1f}deg "
                    f"diff={diff:.1f}deg speed={speed} "
                    f"motors=({left_speed},{right_speed})"
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
