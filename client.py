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

MY_ID = 39  # anpassen, falls deine Tracking-ID anders ist

# Spielfeld
X_MIN = 0.0
X_MAX = 2.0
Y_MIN = 0.0
Y_MAX = 1.0

# Zielgenauigkeit
ARRIVAL_DISTANCE = 0.07  # 7 cm

# Geschwindigkeit
FAST_SPEED = 700
SLOW_SPEED = 350
TURN_SPEED = 250

# Ab wann langsam fahren
SLOW_DISTANCE = 0.25

# Winkel-/Lenkparameter
ANGLE_TOLERANCE = 8.0
STEER_GAIN = 4.0
MAX_CORRECTION = 250

# WICHTIG:
# Falls der Roboter 180° falsch orientiert ist, auf 180 lassen.
# Falls er komplett falsch fährt, teste 0, 90, -90, 180.
ANGLE_OFFSET = 0

# Falls er beim Drehen in die falsche Richtung dreht: auf -1 setzen
TURN_SIGN = 1

# Falls er beim Fahren vom Ziel weglenkt: auf -1 setzen
STEERING_SIGN = 1

robot_positions = {}

pipuck = PiPuck(epuck_version=2)


# =========================
# HELPERS
# =========================

def clamp_speed(v):
    return max(-1024, min(1024, int(v)))


def normalize_angle(angle):
    return angle % 360


def angle_diff(target, current):
    """
    Gibt kürzeste Winkeldifferenz zurück.
    Bereich: -180 bis +180
    """
    return (target - current + 180) % 360 - 180


def distance_xy(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


def angle_to_point(x, y, tx, ty):
    dx = tx - x
    dy = ty - y

    raw_angle = math.degrees(math.atan2(dy, dx))
    return normalize_angle(raw_angle + ANGLE_OFFSET)


def valid_coordinate(x, y):
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX


# =========================
# MOTOR CONTROL
# =========================

def set_motors(left, right):
    pipuck.epuck.set_motor_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )


def stop():
    set_motors(0, 0)


def turn_towards(current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)

    if abs(diff) <= ANGLE_TOLERANCE:
        stop()
        return True, diff

    if TURN_SIGN * diff > 0:
        set_motors(TURN_SPEED, -TURN_SPEED)
    else:
        set_motors(-TURN_SPEED, TURN_SPEED)

    return False, diff


def drive_towards(current_angle, target_angle, speed):
    diff = angle_diff(target_angle, current_angle)

    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left = speed - correction
    right = speed + correction

    # Beide Räder vorwärts halten
    left = max(100, left)
    right = max(100, right)

    set_motors(left, right)

    return diff, clamp_speed(left), clamp_speed(right)


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
# INPUT
# =========================

def ask_target():
    while True:
        try:
            x = float(input("\nZiel-X eingeben [0.0 - 2.0]: "))
            y = float(input("Ziel-Y eingeben [0.0 - 1.0]: "))

            if not valid_coordinate(x, y):
                print("Koordinate außerhalb des Spielfelds. Nochmal.")
                continue

            return x, y

        except ValueError:
            print("Ungültige Eingabe. Bitte Zahlen eingeben, z.B. 1.25")


# =========================
# DRIVE TO TARGET
# =========================

def drive_to_coordinate(target_x, target_y):
    print(f"\nFahre zu Ziel: x={target_x:.2f}, y={target_y:.2f}")

    while True:
        state = get_my_state()

        if state is None:
            print("Keine eigene Position. Sichtbare IDs:", list(robot_positions.keys()))
            stop()
            time.sleep(0.2)
            continue

        x = state["x"]
        y = state["y"]
        angle = state["angle"]

        dist = distance_xy(x, y, target_x, target_y)
        target_angle = angle_to_point(x, y, target_x, target_y)
        diff = angle_diff(target_angle, angle)

        if dist <= ARRIVAL_DISTANCE:
            stop()
            print(
                f"Angekommen | pos=({x:.2f}, {y:.2f}) "
                f"ziel=({target_x:.2f}, {target_y:.2f}) "
                f"dist={dist:.3f}"
            )
            return

        # Wenn Winkel stark falsch: erst drehen
        if abs(diff) > 120:
            done, turn_diff = turn_towards(angle, target_angle)
            print(
                f"mode=TURN | pos=({x:.2f},{y:.2f}) "
                f"ziel=({target_x:.2f},{target_y:.2f}) "
                f"dist={dist:.2f} angle={angle:.1f} "
                f"target_angle={target_angle:.1f} diff={turn_diff:.1f}"
            )

        else:
            speed = SLOW_SPEED if dist < SLOW_DISTANCE else FAST_SPEED
            diff, left, right = drive_towards(angle, target_angle, speed)

            print(
                f"mode=DRIVE | pos=({x:.2f},{y:.2f}) "
                f"ziel=({target_x:.2f},{target_y:.2f}) "
                f"dist={dist:.2f} angle={angle:.1f} "
                f"target_angle={target_angle:.1f} diff={diff:.1f} "
                f"L={left} R={right}"
            )

        time.sleep(0.05)


# =========================
# MAIN
# =========================

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    print("Warte auf MQTT-Position...")
    time.sleep(1.0)

    while True:
        target_x, target_y = ask_target()
        drive_to_coordinate(target_x, target_y)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()