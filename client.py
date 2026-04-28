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

MY_ID = 39  

X_MIN = 0.0
X_MAX = 2.0
Y_MIN = 0.0
Y_MAX = 1.0

# Distanz-Parameter
TAP_DISTANCE = 0.12        # Bei dieser Distanz gilt der Stoß als erfolgt
RAM_THRESHOLD = 0.25       # Ab hier wird von "Slow" auf "Rammen" umgeschaltet
BACKOFF_DURATION = 0.5     # Kurz zurücksetzen nach dem Rammstoß

# Geschwindigkeiten
APPROACH_SPEED = 280       # Erstes langsames Herantasten
RAM_SPEED = 1000           # Maximale Power beim Rammen
BACKOFF_SPEED = -400

TURN_SPEED = 500
ANGLE_TOLERANCE = 12

TURN_SIGN = 1
STEERING_SIGN = 1
STEER_GAIN = 3.5
MAX_CORRECTION = 200

robot_positions = {}

mode = "WAIT_FOR_ROBOTS"
target_ids = []
target_index = 0
current_target_id = None
tap_start_time = None
backoff_start_time = None

# =========================
# BASIC HELPERS
# =========================

def clamp_speed(v):
    return max(-1024, min(1024, int(v)))

def normalize_angle(a):
    return a % 360

def angle_diff(target, current):
    return (target - current + 180) % 360 - 180

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle_to_target(x, y, tx, ty):
    dx = tx - x
    dy = ty - y
    return normalize_angle(math.degrees(math.atan2(dy, dx)))

# =========================
# MQTT
# =========================

def on_connect(client, userdata, flags, rc):
    print(f"Connected to Broker ({rc})")
    client.subscribe("robot_pos/all")

def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        pass

def get_robot_state(robot_id):
    rid = str(robot_id)
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

def get_my_state():
    return get_robot_state(MY_ID)

def get_visible_target_ids():
    """Identifiziert alle anderen IDs auf dem Feld."""
    ids = []
    for rid in robot_positions.keys():
        if int(rid) == MY_ID:
            continue
        ids.append(int(rid))
    return sorted(ids)

# =========================
# ROBOT CONTROL
# =========================

pipuck = PiPuck(epuck_version=2)

def stop():
    pipuck.epuck.set_motor_speeds(0, 0)

def drive_forward(speed):
    s = clamp_speed(speed)
    pipuck.epuck.set_motor_speeds(s, s)

def turn_towards(current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)
    if abs(diff) <= ANGLE_TOLERANCE:
        stop()
        return True, diff
    
    speed = TURN_SPEED if TURN_SIGN * diff > 0 else -TURN_SPEED
    pipuck.epuck.set_motor_speeds(clamp_speed(speed), clamp_speed(-speed))
    return False, diff

def drive_towards_heading(current_angle, target_angle, speed):
    diff = angle_diff(target_angle, current_angle)
    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))
    
    left = speed - correction
    right = speed + correction
    pipuck.epuck.set_motor_speeds(clamp_speed(left), clamp_speed(right))
    return diff

# =========================
# TARGET LOGIC
# =========================

def select_targets():
    global target_ids, target_index, current_target_id
    target_ids = get_visible_target_ids()
    target_index = 0
    if not target_ids:
        current_target_id = None
        return False
    current_target_id = target_ids[target_index]
    print(f"Ziel-Liste erstellt: {target_ids}")
    return True

def select_next_target():
    global target_index, current_target_id, target_ids
    target_ids = get_visible_target_ids() # Liste auffrischen
    if not target_ids:
        return False
    
    target_index = (target_index + 1) % len(target_ids)
    current_target_id = target_ids[target_index]
    print(f"Nächstes Ziel: ID {current_target_id}")
    return True

# =========================
# MAIN LOOP
# =========================

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    while True:
        my_state = get_my_state()
        if my_state is None:
            stop()
            time.sleep(0.1)
            continue

        x, y, angle = my_state["x"], my_state["y"], my_state["angle"]

        if mode == "WAIT_FOR_ROBOTS":
            if select_targets():
                mode = "TURN_TO_TARGET"
            else:
                stop()
                time.sleep(0.5)

        elif mode == "TURN_TO_TARGET":
            target_state = get_robot_state(current_target_id)
            if not target_state:
                select_next_target()
                continue
            
            target_angle = angle_to_target(x, y, target_state["x"], target_state["y"])
            done, _ = turn_towards(angle, target_angle)
            if done:
                mode = "DRIVE_TO_TARGET"

        elif mode == "DRIVE_TO_TARGET":
            target_state = get_robot_state(current_target_id)
            if not target_state:
                if select_next_target(): mode = "TURN_TO_TARGET"
                else: mode = "WAIT_FOR_ROBOTS"
                continue

            tx, ty = target_state["x"], target_state["y"]
            d = distance(x, y, tx, ty)
            target_angle = angle_to_target(x, y, tx, ty)

            if d <= TAP_DISTANCE:
                tap_start_time = time.time()
                mode = "TAP"
                continue

            # RAMM-LOGIK: Erst langsam, dann Vollgas
            if d > RAM_THRESHOLD:
                speed = APPROACH_SPEED
                status = "Approaching slowly..."
            else:
                speed = RAM_SPEED
                status = "RAMMING!"
            
            drive_towards_heading(angle, target_angle, speed)
            print(f"Target {current_target_id} | Dist: {d:.2f}m | {status}")

        elif mode == "TAP":
            # Während des eigentlichen Kontakts Speed halten
            drive_forward(RAM_SPEED)
            if time.time() - tap_start_time >= 0.2:
                backoff_start_time = time.time()
                mode = "BACKOFF"

        elif mode == "BACKOFF":
            drive_forward(BACKOFF_SPEED)
            if time.time() - backoff_start_time >= BACKOFF_DURATION:
                stop()
                if select_next_target():
                    mode = "TURN_TO_TARGET"
                else:
                    mode = "WAIT_FOR_ROBOTS"

        time.sleep(0.05)

except KeyboardInterrupt:
    print("User Stop")
finally:
    stop()
    client.loop_stop()
    client.disconnect()