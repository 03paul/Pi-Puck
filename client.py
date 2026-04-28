import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 39  # anpassen

X_MIN = 0.0
X_MAX = 2.0
Y_MIN = 0.0
Y_MAX = 1.0

SAFE_MARGIN = 0.20

FORWARD_SPEED = 650
TURN_LEFT_SPEED = -500
TURN_RIGHT_SPEED = 500

ANGLE_TOLERANCE = 12

robot_positions = {}

mode = "GO_STRAIGHT"
target_angle = None


def clamp_speed(v):
    return max(-1024, min(1024, int(v)))


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


def normalize_angle(angle):
    return angle % 360


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


def drive_forward():
    pipuck.epuck.set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED)


def turn_towards(current_angle, desired_angle):
    diff = angle_diff(desired_angle, current_angle)

    if abs(diff) <= ANGLE_TOLERANCE:
        stop()
        return True, diff

    # Vorzeichen bei deinem Roboter offenbar invertiert:
    # diff > 0 => mit rechter Rückwärtsbewegung drehen
    if diff > 0:
        left = TURN_RIGHT_SPEED
        right = TURN_LEFT_SPEED
    else:
        left = TURN_LEFT_SPEED
        right = TURN_RIGHT_SPEED

    pipuck.epuck.set_motor_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )

    return False, diff


def near_wall(x, y):
    return (
        x <= X_MIN + SAFE_MARGIN or
        x >= X_MAX - SAFE_MARGIN or
        y <= Y_MIN + SAFE_MARGIN or
        y >= Y_MAX - SAFE_MARGIN
    )


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

        if mode == "GO_STRAIGHT":
            if near_wall(x, y):
                stop()
                time.sleep(0.2)

                target_angle = normalize_angle(angle + 180)
                mode = "TURN_180"

                print(f"Wall reached. Turning from {angle:.1f} to {target_angle:.1f}")
            else:
                drive_forward()

            print(
                f"mode={mode} | x={x:.2f}, y={y:.2f}, "
                f"angle={angle:.1f}, target={target_angle}"
            )

        elif mode == "TURN_180":
            done, diff = turn_towards(angle, target_angle)

            print(
                f"mode={mode} | x={x:.2f}, y={y:.2f}, "
                f"angle={angle:.1f}, target={target_angle:.1f}, diff={diff:.1f}"
            )

            if done:
                print("180 turn done. Driving forward.")
                mode = "GO_STRAIGHT"
                time.sleep(0.2)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()