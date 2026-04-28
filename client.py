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

SAFE_MARGIN = 0.20     # gewünschter Abstand zum Rand
HARD_MARGIN = 0.05     # Notfallzone
OBSTACLE_AVOIDANCE_MARGIN = 0.30

BASE_SPEED = 350
STEER_GAIN = 2.0
MAX_CORRECTION = 100

STEERING_SIGN = 1

robot_positions = {}

mode = "GO_STRAIGHT"
start_angle = None
current_wall = None


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

def get_closest_obstacle(my_x, my_y):
    closest_dist = float('inf')
    for rid, data in robot_positions.items():
        if rid == str(MY_ID): continue
        x = data["position"][0]
        y = data["position"][1]
        dist = ((x - my_x)**2 + (y - my_y)**2)**0.5
        if dist < closest_dist:
            closest_dist = dist
    return closest_dist


# ================= ROBOT =================

pipuck = PiPuck(epuck_version=2)

def stop():
    pipuck.epuck.set_motor_speeds(0, 0)

def drive_heading(current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)

    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left = int(BASE_SPEED - correction)
    right = int(BASE_SPEED + correction)

    # 🔴 NIE Rückwärts → verhindert Kreis-Spin
    left = max(200, left)
    right = max(200, right)

    pipuck.epuck.set_motor_speeds(left, right)

    return diff, left, right


# ================= SAFETY =================

def safety_override(x, y):
    """
    Wenn zu nah am Rand → sofort ins Feld zurücklenken
    """

    if x < X_MIN + HARD_MARGIN:
        return 0      # nach rechts
    if x > X_MAX - HARD_MARGIN:
        return 180    # nach links
    if y < Y_MIN + HARD_MARGIN:
        return 90     # nach oben
    if y > Y_MAX - HARD_MARGIN:
        return 270    # nach unten

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
    """
    Uhrzeigersinn entlang Rand.
    Proportional-Regler (PD) hält den Roboter auf exakt SAFE_MARGIN Abstand.
    """
    STEER_P = 150  # Korrektur-Winkel pro 1m Abweichung

    if wall == "bottom":
        # Fahrtrichtung: 0 (Rechts), Wunschlinie: y = Y_MIN + SAFE_MARGIN
        error = (Y_MIN + SAFE_MARGIN) - y
        error = max(-0.5, min(0.5, error))
        target = (0 + error * STEER_P) % 360
        if x >= X_MAX - SAFE_MARGIN:
            return "right", 90
        return "bottom", target

    if wall == "right":
        # Fahrtrichtung: 90 (Oben), Wunschlinie: x = X_MAX - SAFE_MARGIN
        error = x - (X_MAX - SAFE_MARGIN)
        error = max(-0.5, min(0.5, error))
        target = (90 + error * STEER_P) % 360
        if y >= Y_MAX - SAFE_MARGIN:
            return "top", 180
        return "right", target

    if wall == "top":
        # Fahrtrichtung: 180 (Links), Wunschlinie: y = Y_MAX - SAFE_MARGIN
        error = y - (Y_MAX - SAFE_MARGIN)
        error = max(-0.5, min(0.5, error))
        target = (180 + error * STEER_P) % 360
        if x <= X_MIN + SAFE_MARGIN:
            return "left", 270
        return "top", target

    if wall == "left":
        # Fahrtrichtung: 270 (Unten), Wunschlinie: x = X_MIN + SAFE_MARGIN
        error = (X_MIN + SAFE_MARGIN) - x
        error = max(-0.5, min(0.5, error))
        target = (270 + error * STEER_P) % 360
        if y <= Y_MIN + SAFE_MARGIN:
            return "bottom", 0
        return "left", target

    return wall, 0


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

        # 🔴 HARD SAFETY FIRST
        safety_target = safety_override(x, y)
        closest_obstacle_dist = get_closest_obstacle(x, y)

        if closest_obstacle_dist < OBSTACLE_AVOIDANCE_MARGIN:
            target = (angle + 180) % 360  # Wende um 180 Grad
            mode = "OBSTACLE_AVOIDANCE"
        elif safety_target is not None:
            target = safety_target
            mode = "SAFETY"

        else:
            # Falls wir gerade aus einer Notfallzone kommen, gehe zurück in den Normalzustand
            if mode in ["SAFETY", "OBSTACLE_AVOIDANCE"]:
                mode = "FOLLOW_WALL" if current_wall else "GO_STRAIGHT"

            if start_angle is None:
                start_angle = angle

            wall = detect_wall(x, y)

            if mode == "GO_STRAIGHT":
                target = start_angle

                if wall is not None:
                    mode = "FOLLOW_WALL"
                    current_wall = wall

            if mode == "FOLLOW_WALL":
                current_wall, target = wall_follow_target(current_wall, x, y)

        diff, left, right = drive_heading(angle, target)

        print(
            f"mode={mode} | x={x:.2f}, y={y:.2f}, "
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