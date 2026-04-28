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

SAFE_MARGIN = 0.25
HARD_MARGIN = 0.12

BASE_SPEED = 850
STEER_GAIN = 2.5
MAX_CORRECTION = 120

STEERING_SIGN = 1

robot_positions = {}

mode = "GO_STRAIGHT"
start_angle = None
current_wall = None


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


def drive_heading(current_angle, target_angle, safety=False):
    diff = angle_diff(target_angle, current_angle)

    if safety:
        if diff > 0:
            left = 650
            right = -350
        else:
            left = -350
            right = 650

        left = clamp_speed(left)
        right = clamp_speed(right)

        pipuck.epuck.set_motor_speeds(left, right)
        return diff, left, right

    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left = BASE_SPEED - correction
    right = BASE_SPEED + correction

    left = clamp_speed(left)
    right = clamp_speed(right)

    pipuck.epuck.set_motor_speeds(left, right)
    return diff, left, right


def safety_override(x, y):
    if x < X_MIN + HARD_MARGIN:
        return 0

    if x > X_MAX - HARD_MARGIN:
        return 180

    if y < Y_MIN + HARD_MARGIN:
        return 90

    if y > Y_MAX - HARD_MARGIN:
        return 270

    return None


def detect_wall(x, y):
    if x <= X_MIN + SAFE_MARGIN:
        return "left"

    if x >= X_MAX - SAFE_MARGIN:
        return "right"

    if y <= Y_MIN + SAFE_MARGIN:
        return "bottom"

    if y >= Y_MAX - SAFE_MARGIN:
        return "top"

    return None


def wall_follow_target(wall, x, y):
    # Uhrzeigersinn
    # bottom -> right
    # right  -> up
    # top    -> left
    # left   -> down

    if wall == "bottom":
        if x >= X_MAX - SAFE_MARGIN:
            return "right", 90
        return "bottom", 0

    if wall == "right":
        if y >= Y_MAX - SAFE_MARGIN:
            return "top", 180
        return "right", 90

    if wall == "top":
        if x <= X_MIN + SAFE_MARGIN:
            return "left", 270
        return "top", 180

    if wall == "left":
        if y <= Y_MIN + SAFE_MARGIN:
            return "bottom", 0
        return "left", 270

    return wall, 0


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

        safety_target = safety_override(x, y)

        if safety_target is not None:
            mode = "SAFETY"
            target = safety_target

        else:
            if start_angle is None:
                start_angle = angle
                print("Start angle:", start_angle)

            wall = detect_wall(x, y)

            if mode == "SAFETY":
                mode = "FOLLOW_WALL"
                current_wall = wall if wall is not None else current_wall

            if mode == "GO_STRAIGHT":
                target = start_angle

                if wall is not None:
                    mode = "FOLLOW_WALL"
                    current_wall = wall
                    print("Hit virtual wall:", current_wall)

            elif mode == "FOLLOW_WALL":
                if current_wall is None:
                    current_wall = wall

                current_wall, target = wall_follow_target(current_wall, x, y)

        diff, left, right = drive_heading(
            angle,
            target,
            safety=(mode == "SAFETY")
        )

        print(
            f"mode={mode} | wall={current_wall} | "
            f"x={x:.2f}, y={y:.2f}, angle={angle:.1f}, "
            f"target={target:.1f}, diff={diff:.1f}, "
            f"L={left}, R={right}"
        )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()