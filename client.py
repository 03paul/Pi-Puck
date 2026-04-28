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

MY_ID = 38  # ANPASSEN: ID aus Dashboard/MQTT, nicht zwingend pi-puck39

# Spielfeldgrenzen aus euren MQTT-Werten grob geschätzt
X_MIN = 0.45
X_MAX = 1.85
Y_MIN = 0.25
Y_MAX = 1.00

MARGIN = 0.12          # Abstand zum Rand
BASE_SPEED = 350
TURN_SPEED = 220
LOOP_DELAY = 0.1

robot_positions = {}

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
        print("Invalid JSON:", msg.payload)

# =========================
# HELPERS
# =========================

def angle_diff(target, current):
    """
    Gibt kleinste Winkeldifferenz in Grad zurück: -180 bis +180
    """
    return (target - current + 180) % 360 - 180

def get_my_state():
    rid = str(MY_ID)

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

# =========================
# ROBOT CONTROL
# =========================

pipuck = PiPuck(epuck_version=2)

def stop():
    pipuck.epuck.set_motor_speeds(0, 0)

def drive_forward():
    pipuck.epuck.set_motor_speeds(BASE_SPEED, BASE_SPEED)

def turn_left():
    pipuck.epuck.set_motor_speeds(-TURN_SPEED, TURN_SPEED)

def turn_right():
    pipuck.epuck.set_motor_speeds(TURN_SPEED, -TURN_SPEED)

def steer_to_angle(current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)

    if abs(diff) < 12:
        drive_forward()
    elif diff > 0:
        turn_left()
    else:
        turn_right()

# =========================
# EDGE FOLLOWING LOGIC
# =========================

def get_target_angle(x, y):
    """
    Fährt gegen den Uhrzeigersinn am Rand entlang.

    Winkelannahme:
    0°   = nach rechts / +x
    90°  = nach oben / +y
    180° = nach links / -x
    270° = nach unten / -y
    """

    near_left = x <= X_MIN + MARGIN
    near_right = x >= X_MAX - MARGIN
    near_bottom = y <= Y_MIN + MARGIN
    near_top = y >= Y_MAX - MARGIN

    # Wenn noch nicht am Rand: zum unteren Rand fahren
    if not (near_left or near_right or near_bottom or near_top):
        return 270

    # Unten: nach rechts fahren
    if near_bottom and not near_right:
        return 0

    # Rechts: nach oben fahren
    if near_right and not near_top:
        return 90

    # Oben: nach links fahren
    if near_top and not near_left:
        return 180

    # Links: nach unten fahren
    if near_left and not near_bottom:
        return 270

    # Ecke: weiterdrehen
    if near_bottom and near_right:
        return 90
    if near_right and near_top:
        return 180
    if near_top and near_left:
        return 270
    if near_left and near_bottom:
        return 0

    return 0

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
        state = get_my_state()

        if state is None:
            print("No position yet. Visible IDs:", list(robot_positions.keys()))
            stop()
            time.sleep(0.3)
            continue

        x = state["x"]
        y = state["y"]
        angle = state["angle"]

        target_angle = get_target_angle(x, y)

        print(
            f"ID={MY_ID} | x={x:.2f}, y={y:.2f}, "
            f"angle={angle:.1f}, target={target_angle}"
        )

        steer_to_angle(angle, target_angle)

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("Stopping robot...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()