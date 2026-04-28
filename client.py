import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 38  # anpassen

X_MIN = 0.0
X_MAX = 2.0
Y_MIN = 0.0
Y_MAX = 1.0

MARGIN = 0.12

BASE_SPEED = 300
STEER_GAIN = 4.0
MAX_CORRECTION = 220

# Wenn er falsch herum korrigiert: auf -1 ändern
STEERING_SIGN = 1

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

def drive_heading(current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)

    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left = int(BASE_SPEED - correction)
    right = int(BASE_SPEED + correction)

    pipuck.epuck.set_motor_speeds(left, right)

    return diff, left, right

def nearest_edge_target(x, y):
    d_left = abs(x - X_MIN)
    d_right = abs(X_MAX - x)
    d_bottom = abs(y - Y_MIN)
    d_top = abs(Y_MAX - y)

    distances = {
        "left": d_left,
        "right": d_right,
        "bottom": d_bottom,
        "top": d_top
    }

    return min(distances, key=distances.get)

def get_target_angle(x, y):
    near_left = x <= X_MIN + MARGIN
    near_right = x >= X_MAX - MARGIN
    near_bottom = y <= Y_MIN + MARGIN
    near_top = y >= Y_MAX - MARGIN

    # Noch an keinem Rand: zum nächsten Rand fahren
    if not (near_left or near_right or near_bottom or near_top):
        edge = nearest_edge_target(x, y)

        if edge == "right":
            return 0
        if edge == "top":
            return 90
        if edge == "left":
            return 180
        if edge == "bottom":
            return 270

    # Uhrzeigersinn am Rand entlang:
    # unten: nach rechts
    if near_bottom and not near_right:
        return 0

    # rechts: nach oben
    if near_right and not near_top:
        return 90

    # oben: nach links
    if near_top and not near_left:
        return 180

    # links: nach unten
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

        target = get_target_angle(x, y)
        diff, left, right = drive_heading(angle, target)

        print(
            f"ID={MY_ID} | x={x:.2f}, y={y:.2f}, "
            f"angle={angle:.1f}, target={target}, "
            f"diff={diff:.1f}, L={left}, R={right}"
        )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()