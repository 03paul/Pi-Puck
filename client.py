import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

# =========================
# CONFIG
# =========================

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 39  # eigene Tracking-ID anpassen

X_MIN = 0.0
X_MAX = 2.0
Y_MIN = 0.0
Y_MAX = 1.0

# Abstand, ab dem Ziel als erreicht gilt
TAP_DISTANCE = 0.13

# Nach dem Antippen kurz zurücksetzen
BACKOFF_DURATION = 0.45

# Motorparameter
FORWARD_SPEED_FAST = 650
FORWARD_SPEED_SLOW = 350
TAP_SPEED = 280
BACKOFF_SPEED = -350

TURN_SPEED = 500
ANGLE_TOLERANCE = 14

# Falls er beim Drehen falsch herum dreht: auf -1 setzen
TURN_SIGN = 1

# Falls er beim Lenken falsch herum korrigiert: auf -1 setzen
STEERING_SIGN = 1

STEER_GAIN = 3.0
MAX_CORRECTION = 180

robot_positions = {}

mode = "WAIT_FOR_ROBOTS"
target_ids = []
target_index = 0
current_target_id = None
tap_start_time = None
backoff_start_time = None


# =========================
# BASIC HELPERS
# =========================

def clamp_speed(v):
    return max(-1024, min(1024, int(v)))


def normalize_angle(a):
    return a % 360


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def angle_to_target(x, y, tx, ty):
    dx = tx - x
    dy = ty - y

    # Annahme:
    # 0°   = +x / rechts
    # 90°  = +y / oben
    # 180° = -x / links
    # 270° = -y / unten
    return normalize_angle(math.degrees(math.atan2(dy, dx)))


# =========================
# MQTT
# =========================

def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe("robot_pos/all")


def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        print("Invalid JSON")


def get_robot_state(robot_id):
    rid = str(robot_id)

    if rid not in robot_positions:
        return None

    data = robot_positions[rid]

    if "position" not in data or "angle" not in data:
        return None

    return {
        "x": data["position"][0],
        "y": data["position"][1],
        "angle": data["angle"]
    }


def get_my_state():
    return get_robot_state(MY_ID)


def get_visible_target_ids():
    ids = []

    for rid in robot_positions.keys():
        if str(rid) == str(MY_ID):
            continue
        ids.append(int(rid))

    return sorted(ids)


# =========================
# ROBOT CONTROL
# =========================

pipuck = PiPuck(epuck_version=2)


def stop():
    pipuck.epuck.set_motor_speeds(0, 0)


def drive_forward(speed):
    speed = clamp_speed(speed)
    pipuck.epuck.set_motor_speeds(speed, speed)


def drive_backward(speed=BACKOFF_SPEED):
    speed = clamp_speed(speed)
    pipuck.epuck.set_motor_speeds(speed, speed)


def turn_towards(current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)

    if abs(diff) <= ANGLE_TOLERANCE:
        stop()
        return True, diff

    if TURN_SIGN * diff > 0:
        left = TURN_SPEED
        right = -TURN_SPEED
    else:
        left = -TURN_SPEED
        right = TURN_SPEED

    pipuck.epuck.set_motor_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )

    return False, diff


def drive_towards_heading(current_angle, target_angle, speed):
    diff = angle_diff(target_angle, current_angle)

    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left = speed - correction
    right = speed + correction

    # Beide Räder vorwärts halten, damit er nicht auf der Stelle kreiselt
    left = max(120, left)
    right = max(120, right)

    pipuck.epuck.set_motor_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )

    return diff, clamp_speed(left), clamp_speed(right)


# =========================
# TARGET LOGIC
# =========================

def select_targets():
    global target_ids, target_index, current_target_id

    target_ids = get_visible_target_ids()
    target_index = 0

    if len(target_ids) == 0:
        current_target_id = None
        return False

    current_target_id = target_ids[target_index]
    print("Identified robot IDs:", target_ids)
    print("First target:", current_target_id)
    return True


def select_next_target():
    global target_index, current_target_id, target_ids

    # Liste aktualisieren, falls Roboter verschwinden/auftauchen
    target_ids = get_visible_target_ids()

    if len(target_ids) == 0:
        current_target_id = None
        return False

    target_index = (target_index + 1) % len(target_ids)
    current_target_id = target_ids[target_index]

    print("Next target:", current_target_id)
    return True


def target_available():
    if current_target_id is None:
        return False
    return get_robot_state(current_target_id) is not None


# =========================
# MAIN
# =========================

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    while True:
        my_state = get_my_state()

        if my_state is None:
            print("No own position yet. Visible IDs:", list(robot_positions.keys()))
            stop()
            time.sleep(0.2)
            continue

        x = my_state["x"]
        y = my_state["y"]
        angle = my_state["angle"]

        if mode == "WAIT_FOR_ROBOTS":
            if select_targets():
                mode = "TURN_TO_TARGET"
            else:
                print("No other robots visible.")
                stop()
                time.sleep(0.5)
                continue

        elif mode == "TURN_TO_TARGET":
            if not target_available():
                print("Target disappeared. Selecting next.")
                if not select_next_target():
                    mode = "WAIT_FOR_ROBOTS"
                continue

            target_state = get_robot_state(current_target_id)
            tx = target_state["x"]
            ty = target_state["y"]

            target_angle = angle_to_target(x, y, tx, ty)
            done, diff = turn_towards(angle, target_angle)

            d = distance(x, y, tx, ty)

            print(
                f"mode={mode} | target={current_target_id} | "
                f"my=({x:.2f},{y:.2f}) target=({tx:.2f},{ty:.2f}) "
                f"d={d:.2f} angle={angle:.1f} target_angle={target_angle:.1f} diff={diff:.1f}"
            )

            if done:
                mode = "DRIVE_TO_TARGET"

        elif mode == "DRIVE_TO_TARGET":
            if not target_available():
                print("Target disappeared. Selecting next.")
                if select_next_target():
                    mode = "TURN_TO_TARGET"
                else:
                    mode = "WAIT_FOR_ROBOTS"
                continue

            target_state = get_robot_state(current_target_id)
            tx = target_state["x"]
            ty = target_state["y"]

            d = distance(x, y, tx, ty)
            target_angle = angle_to_target(x, y, tx, ty)

            if d <= TAP_DISTANCE:
                print(f"Reached target {current_target_id}. Tapping.")
                tap_start_time = time.time()
                mode = "TAP"
                continue

            # Je näher, desto langsamer
            if d < 0.35:
                speed = FORWARD_SPEED_SLOW
            else:
                speed = FORWARD_SPEED_FAST

            diff, left, right = drive_towards_heading(angle, target_angle, speed)

            print(
                f"mode={mode} | target={current_target_id} | "
                f"my=({x:.2f},{y:.2f}) target=({tx:.2f},{ty:.2f}) "
                f"d={d:.2f} angle={angle:.1f} target_angle={target_angle:.1f} "
                f"diff={diff:.1f} L={left} R={right}"
            )

        elif mode == "TAP":
            drive_forward(TAP_SPEED)

            elapsed = time.time() - tap_start_time

            print(
                f"mode={mode} | target={current_target_id} | "
                f"tapping... {elapsed:.2f}/0.25s"
            )

            if elapsed >= 0.25:
                stop()
                time.sleep(0.1)
                backoff_start_time = time.time()
                mode = "BACKOFF"

        elif mode == "BACKOFF":
            drive_backward(BACKOFF_SPEED)

            elapsed = time.time() - backoff_start_time

            print(
                f"mode={mode} | target={current_target_id} | "
                f"backing off... {elapsed:.2f}/{BACKOFF_DURATION:.2f}s"
            )

            if elapsed >= BACKOFF_DURATION:
                stop()
                time.sleep(0.1)

                if select_next_target():
                    mode = "TURN_TO_TARGET"
                else:
                    mode = "WAIT_FOR_ROBOTS"

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()