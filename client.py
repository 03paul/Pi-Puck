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

# Distanzen (in Metern, falls das Tracking in Metern ist)
RAM_DISTANCE = 0.35        # Ab 35cm wird der "Charge" (Tempo 1000) ausgelöst
STOP_DISTANCE = 0.10       # Wenn Ziel erreicht

# Geschwindigkeiten
SPEED_APPROACH = 400       # Ruhiges Heranfahren
SPEED_CHARGE = 1000        # Maximaler Ramm-Speed
SPEED_BACKOFF = -400

# Toleranzen
TURN_PRECISION = 5         # Grad-Toleranz im Stand
STEER_GAIN = 4.0           # Wie stark er während der Fahrt korrigiert

robot_positions = {}
mode = "SELECT_TARGET"
target_ids = []
target_index = 0
current_target_id = None
action_start_time = None

# =========================
# GEOMETRY HELPERS
# =========================

def get_dist_and_angle(source, target):
    dx = target["x"] - source["x"]
    dy = target["y"] - source["y"]
    dist = math.sqrt(dx**2 + dy**2)
    # Winkel berechnen (0° ist +x)
    target_angle = math.degrees(math.atan2(dy, dx)) % 360
    return dist, target_angle

def get_angle_diff(target, current):
    diff = (target - current + 180) % 360 - 180
    return diff

# =========================
# CONTROL
# =========================
pipuck = PiPuck(epuck_version=2)

def set_motors(l, r):
    pipuck.epuck.set_motor_speeds(int(max(-1024, min(1024, l))), 
                                  int(max(-1024, min(1024, r))))

def stop():
    set_motors(0, 0)

# =========================
# MQTT
# =========================
def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except: pass

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.subscribe("robot_pos/all")
client.loop_start()

def get_state(rid):
    rid_s = str(rid)
    if rid_s in robot_positions:
        d = robot_positions[rid_s]
        return {"x": d["position"][0], "y": d["position"][1], "angle": d["angle"]}
    return None

# =========================
# MAIN LOOP
# =========================
try:
    while True:
        my = get_state(MY_ID)
        if not my:
            time.sleep(0.1)
            continue

        if mode == "SELECT_TARGET":
            # Alle IDs außer der eigenen holen
            target_ids = sorted([int(rid) for rid in robot_positions.keys() if int(rid) != MY_ID])
            if target_ids:
                current_target_id = target_ids[target_index % len(target_ids)]
                print(f"--- Ziel erfasst: ID {current_target_id} ---")
                mode = "AIM"
            else:
                stop()

        elif mode == "AIM":
            tar = get_state(current_target_id)
            if not tar: 
                mode = "SELECT_TARGET"
                continue
            
            _, target_angle = get_dist_and_angle(my, tar)
            diff = get_angle_diff(target_angle, my["angle"])
            
            if abs(diff) < TURN_PRECISION:
                stop()
                print("Lock-on bestätigt. Fahre an.")
                mode = "APPROACH"
            else:
                # Auf der Stelle drehen
                speed = 350 if diff > 0 else -350
                set_motors(speed, -speed)

        elif mode == "APPROACH":
            tar = get_state(current_target_id)
            if not tar:
                mode = "SELECT_TARGET"
                continue
                
            dist, target_angle = get_dist_and_angle(my, tar)
            diff = get_angle_diff(target_angle, my["angle"])
            
            if dist < RAM_DISTANCE:
                print("!!! CHARGE !!!")
                mode = "CHARGE"
                continue
            
            # Sanfte Korrektur während der Fahrt (kein Stoppen!)
            correction = diff * STEER_GAIN
            set_motors(SPEED_APPROACH - correction, SPEED_APPROACH + correction)

        elif mode == "CHARGE":
            # Im Charge-Modus ignorieren wir feine Kursänderungen fast komplett
            # Wir fahren einfach mit Gewalt auf die letzte Position
            tar = get_state(current_target_id)
            if not tar:
                dist = 0 # Falls Target weg, brechen wir ab
            else:
                dist, _ = get_dist_and_angle(my, tar)

            if dist < STOP_DISTANCE:
                print("Einschlag!")
                action_start_time = time.time()
                mode = "IMPACT_HOLD"
            else:
                # Volles Rohr
                set_motors(SPEED_CHARGE, SPEED_CHARGE)

        elif mode == "IMPACT_HOLD":
            # Kurz gegen den Gegner drücken, um Treffer sicherzustellen
            set_motors(SPEED_CHARGE, SPEED_CHARGE)
            if time.time() - action_start_time > 0.3:
                action_start_time = time.time()
                mode = "BACKOFF"

        elif mode == "BACKOFF":
            set_motors(SPEED_BACKOFF, SPEED_BACKOFF)
            if time.time() - action_start_time > 0.6:
                stop()
                target_index += 1 # Nächster Roboter in der Liste
                mode = "SELECT_TARGET"

        time.sleep(0.02)

except KeyboardInterrupt:
    stop()
finally:
    client.loop_stop()