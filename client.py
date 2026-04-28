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

# Distanzen
RAM_DISTANCE = 0.40        # Ab 40cm wird die Flugbahn "eingeloggt"
STOP_DISTANCE = 0.12       # Einschlag-Distanz

# Geschwindigkeiten
SPEED_CHARGE = 1000        
SPEED_APPROACH = 500
SPEED_BACKOFF = -400

# Toleranzen (Trigonometrie)
AIM_THRESHOLD = 2.5        # Nur wenn Abweichung > 2.5°, wird nachjustiert
P_TURN = 6.0               # Proportionalitätsfaktor für sanftes Drehen

robot_positions = {}
mode = "SELECT_TARGET"
target_ids = []
target_index = 0
current_target_id = None
action_start_time = None

# =========================
# TRIGONOMETRY HELPERS
# =========================

def get_target_data(my_pos, tar_pos):
    dx = tar_pos["x"] - my_pos["x"]
    dy = tar_pos["y"] - my_pos["y"]
    dist = math.sqrt(dx**2 + dy**2)
    # Atan2 liefert den exakten Winkel zum Ziel
    target_angle = math.degrees(math.atan2(dy, dx)) % 360
    return dist, target_angle

def get_angle_diff(target, current):
    # Berechnet den kürzesten Weg zum Zielwinkel (-180 bis 180)
    return (target - current + 180) % 360 - 180

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
            time.sleep(0.05)
            continue

        if mode == "SELECT_TARGET":
            target_ids = sorted([int(rid) for rid in robot_positions.keys() if int(rid) != MY_ID])
            if target_ids:
                current_target_id = target_ids[target_index % len(target_ids)]
                print(f"Visier fixiert auf ID: {current_target_id}")
                mode = "AIM"
            else:
                stop()

        elif mode == "AIM":
            tar = get_state(current_target_id)
            if not tar: 
                mode = "SELECT_TARGET"; continue
            
            _, target_angle = get_target_data(my, tar)
            diff = get_angle_diff(target_angle, my["angle"])
            
            if abs(diff) < AIM_THRESHOLD:
                stop()
                print("Winkel geloggt. Starte Angriff.")
                mode = "APPROACH"
            else:
                # Proportionales Drehen: Je kleiner diff, desto langsamer die Motoren
                # Das verhindert das Hin-und-Her-Zappeln (Oszillation)
                turn_speed = diff * P_TURN
                # Mindestgeschwindigkeit, damit er nicht verhungert
                min_speed = 150 if diff > 0 else -150
                final_turn = turn_speed + min_speed
                set_motors(final_turn, -final_turn)

        elif mode == "APPROACH":
            tar = get_state(current_target_id)
            if not tar:
                mode = "SELECT_TARGET"; continue
                
            dist, target_angle = get_target_data(my, tar)
            diff = get_angle_diff(target_angle, my["angle"])
            
            # Wenn wir nah genug sind: Volles Rohr ohne Rücksicht auf Verluste
            if dist < RAM_DISTANCE:
                mode = "CHARGE"
                continue
            
            # Falls wir uns während der Fahrt komplett verrennen (> 30° Abweichung)
            if abs(diff) > 30:
                mode = "AIM"
                continue

            # Während der Fahrt nur sanft korrigieren
            correction = diff * 3.0
            set_motors(SPEED_APPROACH - correction, SPEED_APPROACH + correction)

        elif mode == "CHARGE":
            # Hier wird nicht mehr gelenkt. Wir vertrauen auf die Trigonometrie der Anfahrt.
            # Voller Speed auf beiden Motoren für maximale Wucht.
            set_motors(SPEED_CHARGE, SPEED_CHARGE)
            
            tar = get_state(current_target_id)
            dist = distance = 0
            if tar:
                dist, _ = get_target_data(my, tar)

            if dist < STOP_DISTANCE:
                print("TREFFER!")
                action_start_time = time.time()
                mode = "IMPACT_HOLD"

        elif mode == "IMPACT_HOLD":
            set_motors(SPEED_CHARGE, SPEED_CHARGE)
            if time.time() - action_start_time > 0.3:
                action_start_time = time.time()
                mode = "BACKOFF"

        elif mode == "BACKOFF":
            set_motors(SPEED_BACKOFF, SPEED_BACKOFF)
            if time.time() - action_start_time > 0.6:
                stop()
                target_index += 1
                mode = "SELECT_TARGET"

        time.sleep(0.02)

except KeyboardInterrupt:
    stop()
finally:
    stop()
    client.loop_stop()
    client.disconnect()