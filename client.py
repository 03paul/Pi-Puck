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

# Distanz-Parameter
TAP_DISTANCE = 0.12
RAM_THRESHOLD = 0.30       # Etwas früher auf Rammen schalten für Stabilität
BACKOFF_DURATION = 0.5

# Geschwindigkeiten
APPROACH_SPEED = 300       
RAM_SPEED = 1000           
BACKOFF_SPEED = -400

TURN_SPEED = 450
ANGLE_TOLERANCE = 3        # VIEL PRÄZISER (für gerade Linie)
RE_TURN_THRESHOLD = 15     # Wenn Abweichung > 15°, dann stoppen und neu ausrichten

STEER_GAIN = 5.0           # Aggressivere Korrektur
MAX_CORRECTION = 250

robot_positions = {}
mode = "WAIT_FOR_ROBOTS"
target_ids = []
target_index = 0
current_target_id = None
tap_start_time = None
backoff_start_time = None

# =========================
# HELPERS
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
    return normalize_angle(math.degrees(math.atan2(ty - y, tx - x)))

# =========================
# MQTT & STATE
# =========================

def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except: pass

def get_robot_state(robot_id):
    rid = str(robot_id)
    if rid not in robot_positions: return None
    d = robot_positions[rid]
    return {"x": d["position"][0], "y": d["position"][1], "angle": d["angle"]}

def get_visible_target_ids():
    return sorted([int(rid) for rid in robot_positions.keys() if int(rid) != MY_ID])

# =========================
# CONTROL
# =========================

pipuck = PiPuck(epuck_version=2)

def stop():
    pipuck.epuck.set_motor_speeds(0, 0)

def drive_straight(speed, current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)
    
    # Wenn wir zu weit abweichen, Modus wechseln zum neu Ausrichten
    if abs(diff) > RE_TURN_THRESHOLD:
        return False 

    correction = diff * STEER_GAIN
    left = speed - correction
    right = speed + correction
    pipuck.epuck.set_motor_speeds(clamp_speed(left), clamp_speed(right))
    return True

def turn_towards(current_angle, target_angle):
    diff = angle_diff(target_angle, current_angle)
    if abs(diff) <= ANGLE_TOLERANCE:
        stop()
        return True
    
    # Drehen auf der Stelle
    speed = TURN_SPEED if diff > 0 else -TURN_SPEED
    pipuck.epuck.set_motor_speeds(clamp_speed(speed), clamp_speed(-speed))
    return False

# =========================
# MAIN LOGIC
# =========================

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.subscribe("robot_pos/all")
client.loop_start()

try:
    while True:
        my = get_robot_state(MY_ID)
        if not my:
            stop()
            time.sleep(0.1)
            continue

        if mode == "WAIT_FOR_ROBOTS":
            target_ids = get_visible_target_ids()
            if target_ids:
                target_index = 0
                current_target_id = target_ids[target_index]
                mode = "TURN_TO_TARGET"
            else:
                stop()

        elif mode == "TURN_TO_TARGET":
            tar = get_robot_state(current_target_id)
            if not tar:
                mode = "WAIT_FOR_ROBOTS"
                continue
            
            target_angle = angle_to_target(my["x"], my["y"], tar["x"], tar["y"])
            if turn_towards(my["angle"], target_angle):
                print(f"Ausgerichtet auf ID {current_target_id}. Starte Anflug.")
                mode = "DRIVE_TO_TARGET"

        elif mode == "DRIVE_TO_TARGET":
            tar = get_robot_state(current_target_id)
            if not tar:
                mode = "WAIT_FOR_ROBOTS"
                continue

            d = distance(my["x"], my["y"], tar["x"], tar["y"])
            target_angle = angle_to_target(my["x"], my["y"], tar["x"], tar["y"])

            if d <= TAP_DISTANCE:
                tap_start_time = time.time()
                mode = "TAP"
                continue

            # Geschwindigkeit wählen
            current_speed = RAM_SPEED if d < RAM_THRESHOLD else APPROACH_SPEED
            
            # Fahren und Kurs korrigieren
            on_track = drive_straight(current_speed, my["angle"], target_angle)
            
            if not on_track:
                print("Kursabweichung zu groß! Korrigiere...")
                stop()
                mode = "TURN_TO_TARGET"

        elif mode == "TAP":
            pipuck.epuck.set_motor_speeds(RAM_SPEED, RAM_SPEED)
            if time.time() - tap_start_time >= 0.2:
                backoff_start_time = time.time()
                mode = "BACKOFF"

        elif mode == "BACKOFF":
            pipuck.epuck.set_motor_speeds(BACKOFF_SPEED, BACKOFF_SPEED)
            if time.time() - backoff_start_time >= BACKOFF_DURATION:
                stop()
                # Nächstes Ziel wählen
                target_ids = get_visible_target_ids()
                if target_ids:
                    target_index = (target_index + 1) % len(target_ids)
                    current_target_id = target_ids[target_index]
                    mode = "TURN_TO_TARGET"
                else:
                    mode = "WAIT_FOR_ROBOTS"

        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    stop()
    client.loop_stop()