import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

# New Vers

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 38  # eigene Tracking-ID anpassen

TAP_DISTANCE = 0.13

FAST_SPEED = 650
SLOW_SPEED = 350
TAP_SPEED = 300
BACKOFF_SPEED = -350

STEER_GAIN = 3.0
MAX_CORRECTION = 220

# Falls er vom Ziel weglenkt: auf -1 setzen
STEERING_SIGN = 1

# Falls Zielwinkel systematisch falsch ist, hier später anpassen:
ANGLE_OFFSET = 0.0

robot_positions = {}

mode = "SELECT_TARGET"
target_ids = []
target_index = 0
current_target_id = None
tap_start = None
backoff_start = None


def clamp_speed(v):
    return max(-1024, min(1024, int(v)))


def normalize_angle(a):
    return a % 360


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def distance(x1, y1, x2, y2):
    return math.sqrt((tx - x) ** 2 + (ty - y) ** 2)


def dist(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def angle_to_target(x, y, tx, ty):
    dx = tx - x
    dy = ty - y

    # Standardannahme:
    # 0° = rechts, 90° = oben, 180° = links, 270° = unten
    angle = math.degrees(math.atan2(dy, dx))

    return normalize_angle(angle + ANGLE_OFFSET)


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

    return {
        "x": data["position"][0],
        "y": data["position"][1],
        "angle": data["angle"]
    }


def get_my_state():
    return get_robot_state(MY_ID)


def get_target_ids():
    ids = []
    for rid in robot_positions.keys():
        if str(rid) != str(MY_ID):
            ids.append(int(rid))
    return sorted(ids)


pipuck = PiPuck(epuck_version=2)


def stop():
    pipuck.epuck.set_motor_speeds(0, 0)


def drive_towards(current_angle, target_angle, speed):
    diff = angle_diff(target_angle, current_angle)

    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left = speed - correction
    right = speed + correction

    # Beide Räder bleiben vorwärts, damit er nicht endlos kreiselt
    left = max(120, left)
    right = max(120, right)

    left = clamp_speed(left)
    right = clamp_speed(right)

    pipuck.epuck.set_motor_speeds(left, right)

    return diff, left, right


def drive_forward(speed):
    speed = clamp_speed(speed)
    pipuck.epuck.set_motor_speeds(speed, speed)


def drive_backward():
    pipuck.epuck.set_motor_speeds(
        clamp_speed(BACKOFF_SPEED),
        clamp_speed(BACKOFF_SPEED)
    )


def select_targets():
    global target_ids, target_index, current_target_id

    target_ids = get_target_ids()

    if len(target_ids) == 0:
        current_target_id = None
        return False

    target_index = 0
    current_target_id = target_ids[target_index]

    print("Gefundene IDs:", target_ids)
    print("Visier fixiert auf ID:", current_target_id)

    return True


def select_next_target():
    global target_ids, target_index, current_target_id

    target_ids = get_target_ids()

    if len(target_ids) == 0:
        current_target_id = None
        return False

    target_index = (target_index + 1) % len(target_ids)
    current_target_id = target_ids[target_index]

    print("Nächstes Ziel:", current_target_id)

    return True


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    while True:
        my = get_my_state()

        if my is None:
            print("Keine eigene Position. Sichtbare IDs:", list(robot_positions.keys()))
            stop()
            time.sleep(0.2)
            continue

        x = my["x"]
        y = my["y"]
        angle = my["angle"]

        if mode == "SELECT_TARGET":
            if select_targets():
                mode = "CHASE"
            else:
                print("Keine anderen Roboter sichtbar.")
                stop()
                time.sleep(0.5)
                continue

        elif mode == "CHASE":
            target = get_robot_state(current_target_id)

            if target is None:
                print("Ziel verschwunden.")
                if select_next_target():
                    mode = "CHASE"
                else:
                    mode = "SELECT_TARGET"
                continue

            tx = target["x"]
            ty = target["y"]

            d = dist(x, y, tx, ty)
            target_angle = angle_to_target(x, y, tx, ty)

            if d <= TAP_DISTANCE:
                print(f"Ziel {current_target_id} erreicht. Antippen.")
                tap_start = time.time()
                mode = "TAP"
                continue

            speed = SLOW_SPEED if d < 0.35 else FAST_SPEED

            diff, left, right = drive_towards(angle, target_angle, speed)

            print(
                f"mode=CHASE | target={current_target_id} | "
                f"my=({x:.2f},{y:.2f}) target=({tx:.2f},{ty:.2f}) "
                f"d={d:.2f} angle={angle:.1f} target_angle={target_angle:.1f} "
                f"diff={diff:.1f} L={left} R={right}"
            )

        elif mode == "TAP":
            drive_forward(TAP_SPEED)

            elapsed = time.time() - tap_start

            print(f"mode=TAP | target={current_target_id} | {elapsed:.2f}/0.25s")

            if elapsed >= 0.25:
                stop()
                time.sleep(0.1)
                backoff_start = time.time()
                mode = "BACKOFF"

        elif mode == "BACKOFF":
            drive_backward()

            elapsed = time.time() - backoff_start

            print(f"mode=BACKOFF | {elapsed:.2f}/0.45s")

            if elapsed >= 0.45:
                stop()
                time.sleep(0.1)

                if select_next_target():
                    mode = "CHASE"
                else:
                    mode = "SELECT_TARGET"

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()