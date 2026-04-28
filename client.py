import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

# =========================
# CONFIG
# =========================

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 38  # anpassen

X_MIN = 0.0
X_MAX = 2.0
Y_MIN = 0.0
Y_MAX = 1.0

SAFE_MARGIN = 0.25

FORWARD_SPEED = 1000

TURN_SPEED_LEFT = -700
TURN_SPEED_RIGHT = 700

ANGLE_TOLERANCE = 10

# Nach Bounce kurz Wand-Check ignorieren
BOUNCE_COOLDOWN = 0.45

robot_positions = {}

mode = "DRIVE"
heading = None
last_bounce_time = 0


# =========================
# BASIC HELPERS
# =========================

def clamp_speed(v):
    return max(-1024, min(1024, int(v)))


def normalize_angle(a):
    return a % 360


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


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


# =========================
# ROBOT
# =========================

pipuck = PiPuck(epuck_version=2)


def stop():
    pipuck.epuck.set_motor_speeds(0, 0)


def forward_full_speed():
    pipuck.epuck.set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED)


def turn_to_heading(current_angle, target_heading):
    diff = angle_diff(target_heading, current_angle)

    if abs(diff) <= ANGLE_TOLERANCE:
        stop()
        return True, diff

    # Bei deinem Setup war die Drehrichtung vorher invertiert.
    # Falls er falsch herum dreht, unten den if/else-Block tauschen.
    if diff > 0:
        left = TURN_SPEED_RIGHT
        right = TURN_SPEED_LEFT
    else:
        left = TURN_SPEED_LEFT
        right = TURN_SPEED_RIGHT

    pipuck.epuck.set_motor_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )

    return False, diff


# =========================
# BILLIARD LOGIC
# =========================

def detect_walls(x, y):
    hit_left = x <= X_MIN + SAFE_MARGIN
    hit_right = x >= X_MAX - SAFE_MARGIN
    hit_bottom = y <= Y_MIN + SAFE_MARGIN
    hit_top = y >= Y_MAX - SAFE_MARGIN

    return hit_left, hit_right, hit_bottom, hit_top


def reflect_heading(old_heading, hit_left, hit_right, hit_bottom, hit_top):
    new_heading = old_heading

    # Vertikale Wand: x-Komponente spiegeln
    if hit_left or hit_right:
        new_heading = 180 - new_heading

    # Horizontale Wand: y-Komponente spiegeln
    if hit_bottom or hit_top:
        new_heading = -new_heading

    return normalize_angle(new_heading)


def in_bounce_cooldown():
    return (time.time() - last_bounce_time) < BOUNCE_COOLDOWN


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
            print("No position yet. IDs:", list(robot_positions.keys()))
            stop()
            time.sleep(0.2)
            continue

        x = state["x"]
        y = state["y"]
        angle = state["angle"]

        if heading is None:
            heading = normalize_angle(angle)
            print(f"Initial heading: {heading:.1f}")

        hit_left, hit_right, hit_bottom, hit_top = detect_walls(x, y)

        if mode == "DRIVE":
            if (hit_left or hit_right or hit_bottom or hit_top) and not in_bounce_cooldown():
                old_heading = heading
                heading = reflect_heading(
                    heading,
                    hit_left,
                    hit_right,
                    hit_bottom,
                    hit_top
                )

                last_bounce_time = time.time()
                mode = "TURN_TO_NEW_HEADING"
                stop()

                print(
                    f"BOUNCE | old={old_heading:.1f}, new={heading:.1f} | "
                    f"L={hit_left}, R={hit_right}, B={hit_bottom}, T={hit_top}"
                )

            else:
                forward_full_speed()

            print(
                f"mode={mode} | x={x:.2f}, y={y:.2f}, "
                f"angle={angle:.1f}, heading={heading:.1f}"
            )

        elif mode == "TURN_TO_NEW_HEADING":
            done, diff = turn_to_heading(angle, heading)

            print(
                f"mode={mode} | x={x:.2f}, y={y:.2f}, "
                f"angle={angle:.1f}, heading={heading:.1f}, diff={diff:.1f}"
            )

            if done:
                mode = "ESCAPE_AFTER_BOUNCE"
                last_bounce_time = time.time()
                print("Aligned. Escaping from wall.")

        elif mode == "ESCAPE_AFTER_BOUNCE":
            forward_full_speed()

            elapsed = time.time() - last_bounce_time

            print(
                f"mode={mode} | escape={elapsed:.2f}/{BOUNCE_COOLDOWN:.2f} | "
                f"x={x:.2f}, y={y:.2f}, angle={angle:.1f}, heading={heading:.1f}"
            )

            if elapsed >= BOUNCE_COOLDOWN:
                mode = "DRIVE"

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()