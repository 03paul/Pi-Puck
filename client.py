import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 39

X_MIN = 0.0
X_MAX = 2.0
Y_MIN = 0.0
Y_MAX = 1.0

MARGIN = 0.20

BASE_SPEED = 600
TURN_SPEED = 600

robot_positions = {}

mode = "FORWARD"
target_angle = None


# ================= MQTT =================

def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe("robot_pos/all")

def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except:
        pass


# ================= HELPERS =================

def angle_diff(target, current):
    return (target - current + 180) % 360 - 180

def normalize_angle(a):
    return a % 360

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


# ================= ROBOT =================

pipuck = PiPuck(epuck_version=2)

def stop():
    pipuck.epuck.set_motor_speeds(0, 0)

def forward():
    pipuck.epuck.set_motor_speeds(BASE_SPEED, BASE_SPEED)

def turn_left():
    pipuck.epuck.set_motor_speeds(-TURN_SPEED, TURN_SPEED)

def turn_right():
    pipuck.epuck.set_motor_speeds(TURN_SPEED, -TURN_SPEED)


# ================= WALL DETECTION =================

def hit_wall(x, y):
    if x <= X_MIN + MARGIN:
        return True
    if x >= X_MAX - MARGIN:
        return True
    if y <= Y_MIN + MARGIN:
        return True
    if y >= Y_MAX - MARGIN:
        return True
    return False


# ================= MAIN =================

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    while True:
        state = get_my_state()

        if state is None:
            stop()
            time.sleep(0.2)
            continue

        x = state["x"]
        y = state["y"]
        angle = state["angle"]

        # ================= FORWARD =================
        if mode == "FORWARD":

            if hit_wall(x, y):
                print("🔥 HIT WALL → BOUNCE")

                target_angle = normalize_angle(angle + 180)
                mode = "TURN"

            else:
                forward()

        # ================= TURN =================
        elif mode == "TURN":

            diff = angle_diff(target_angle, angle)

            print(f"TURNING | angle={angle:.1f} target={target_angle:.1f} diff={diff:.1f}")

            if abs(diff) < 10:
                print("✅ TURN DONE")
                mode = "FORWARD"
            else:
                if diff > 0:
                    turn_left()
                else:
                    turn_right()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()