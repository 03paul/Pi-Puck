import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 38  # anpassen

X_MIN = 0.45
X_MAX = 1.85
Y_MIN = 0.25
Y_MAX = 1.00

MARGIN = 0.12

BASE_SPEED = 250
STEER_GAIN = 5.0
MAX_CORRECTION = 180

# Falls er falsch herum lenkt: auf -1 setzen
STEERING_SIGN = -1

robot_positions = {}

def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe("robot_pos/all")

def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        print("Invalid JSON")

def angle_diff(target, current):
    return (target - current + 180) % 360 - 180

def get_my_state():
    rid = str(MY_ID)

    if rid not in robot_positions:
        return None

    data = robot_positions[rid]

    return {
        "x": data["position"][0],
        "y": data["position"][1],
        "angle": data["angle"]
    }

pipuck = PiPuck(epuck_version=2)

def stop():
    pipuck.epuck.set_motor_speeds(0, 0)

def drive_with_heading(current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)

    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left_speed = int(BASE_SPEED - correction)
    right_speed = int(BASE_SPEED + correction)

    pipuck.epuck.set_motor_speeds(left_speed, right_speed)

    return diff, left_speed, right_speed

def get_target_angle(x, y):
    near_left = x <= X_MIN + MARGIN
    near_right = x >= X_MAX - MARGIN
    near_bottom = y <= Y_MIN + MARGIN
    near_top = y >= Y_MAX - MARGIN

    # Noch nicht am Rand: erstmal zum unteren Rand fahren
    if not (near_left or near_right or near_bottom or near_top):
        return 270

    # Unten entlang nach rechts
    if near_bottom and not near_right:
        return 0

    # Rechts entlang nach oben
    if near_right and not near_top:
        return 90

    # Oben entlang nach links
    if near_top and not near_left:
        return 180

    # Links entlang nach unten
    if near_left and not near_bottom:
        return 270

    # Ecken
    if near_bottom and near_right:
        return 90
    if near_right and near_top:
        return 180
    if near_top and near_left:
        return 270
    if near_left and near_bottom:
        return 0

    return 0

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    while True:
        state = get_my_state()

        if state is None:
            print("No position yet. IDs:", list(robot_positions.keys()))
            stop()
            time.sleep(0.2)
            continue

        x = state["x"]
        y = state["y"]
        angle = state["angle"]

        target_angle = get_target_angle(x, y)

        diff, left, right = drive_with_heading(angle, target_angle)

        print(
            f"ID={MY_ID} | x={x:.2f}, y={y:.2f}, "
            f"angle={angle:.1f}, target={target_angle}, "
            f"diff={diff:.1f}, L={left}, R={right}"
        )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()