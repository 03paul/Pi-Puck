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
RAM_TRIGGER_DIST = 0.35    # Ab 35cm Entfernung wird "gelockt" und gerammt
IMPACT_DIST = 0.12         # Wann wir stoppen/zurücksetzen

# Geschwindigkeiten
SPEED_APPROACH = 450       # Langsames Heranfahren
SPEED_RAM = 1000           # Die versprochenen 1000
SPEED_BACKOFF = -400

SAFE_MARGIN = 0.20     # gewünschter Abstand zum Rand
HARD_MARGIN = 0.05     # Notfallzone
OBSTACLE_AVOIDANCE_MARGIN = 0.30

BASE_SPEED = 350
STEER_GAIN = 2.0
MAX_CORRECTION = 100

STEERING_SIGN = 1
=======
# Präzision
AIM_TOLERANCE = 3.5        # Grad-Toleranz (nicht zu eng, sonst zappelt er)
MIN_TURN_SPEED = 200       # Mindestkraft zum Drehen (gegen Reibung)
>>>>>>> 3da9deb2544467e00c068279bedcf06b3c35a346

robot_positions = {}
mode = "SELECT_TARGET"
target_ids = []
target_index = 0
current_target_id = None
timer_start = 0

# =========================
# MATH
# =========================

def get_vector(my_pos, tar_pos):
    dx = tar_pos["x"] - my_pos["x"]
    dy = tar_pos["y"] - my_pos["y"]
    dist = math.sqrt(dx**2 + dy**2)
    # Winkel zum Ziel (0-360)
    angle = math.degrees(math.atan2(dy, dx)) % 360
    return dist, angle

def get_angle_diff(target, current):
    # Kürzester Drehweg zwischen -180 und 180
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
<<<<<<< HEAD
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
=======
    except: pass
>>>>>>> 3da9deb2544467e00c068279bedcf06b3c35a346

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.subscribe("robot_pos/all")
client.loop_start()

def get_state(rid):
    rid_s = str(rid)
    if rid_s in robot_positions:
        d = robot_positions[rid_s]
        # Falls Position als [x, y] kommt
        return {"x": d["position"][0], "y": d["position"][1], "angle": d["angle"]}
    return None

# =========================
# MAIN LOOP
# =========================
try:
    print(f"PiPuck {MY_ID} bereit. Suche Ziele...")
    
    while True:
        my = get_state(MY_ID)
        if not my:
            time.sleep(0.1)
            continue

        if mode == "SELECT_TARGET":
            target_ids = sorted([int(rid) for rid in robot_positions.keys() if int(rid) != MY_ID])
            if target_ids:
                current_target_id = target_ids[target_index % len(target_ids)]
                print(f"Ziel fixiert: ID {current_target_id}")
                mode = "AIM"
            else:
                stop()

<<<<<<< HEAD
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
=======
        elif mode == "AIM":
            tar = get_state(current_target_id)
            if not tar: 
                mode = "SELECT_TARGET"; continue
            
            dist, target_angle = get_vector(my, tar)
            diff = get_angle_diff(target_angle, my["angle"])
            
            # DEBUG
            if int(time.time()*10) % 5 == 0:
                print(f"Aiming: Ist {my['angle']:.1f}°, Ziel {target_angle:.1f}°, Diff {diff:.1f}°")

            if abs(diff) < AIM_TOLERANCE:
                stop()
                print(">>> Winkel fixiert! Starte Anflug.")
                mode = "APPROACH"
            else:
                # Sanftes Drehen mit Mindestgeschwindigkeit gegen Reibung
                turn_speed = diff * 5.0 
                if turn_speed > 0: turn_speed = max(min(turn_speed, 500), MIN_TURN_SPEED)
                else: turn_speed = min(max(turn_speed, -500), -MIN_TURN_SPEED)
                set_motors(turn_speed, -turn_speed)

        elif mode == "APPROACH":
            tar = get_state(current_target_id)
            if not tar: mode = "SELECT_TARGET"; continue
                
            dist, target_angle = get_vector(my, tar)
            diff = get_angle_diff(target_angle, my["angle"])
            
            # Wenn wir im "Charge-Bereich" sind, schalten wir die Lenkung aus!
            if dist < RAM_TRIGGER_DIST:
                print(">>> RAMMEN MIT TEMPO 1000!")
                mode = "CHARGE"
                continue
>>>>>>> 3da9deb2544467e00c068279bedcf06b3c35a346

            # Während der langsamen Fahrt Kurs halten
            # Wir lassen eine größere Toleranz (15°) zu, bevor wir abbrechen
            if abs(diff) > 25:
                print("Kurs verloren, korrigiere...")
                mode = "AIM"
                continue

            correction = diff * 4.0
            set_motors(SPEED_APPROACH - correction, SPEED_APPROACH + correction)

        elif mode == "CHARGE":
            # KEINE BERECHNUNG MEHR. Einfach Vollgas geradeaus.
            set_motors(SPEED_RAM, SPEED_RAM)
            
            tar = get_state(current_target_id)
            if tar:
                dist, _ = get_vector(my, tar)
                if dist < IMPACT_DIST:
                    print("TREFFER!")
                    timer_start = time.time()
                    mode = "IMPACT_HOLD"
            else:
                # Falls Ziel weg, kurz weiterhämmern, dann SELECT
                time.sleep(0.2)
                mode = "BACKOFF"

<<<<<<< HEAD
            if mode == "FOLLOW_WALL":
                current_wall, target = wall_follow_target(current_wall, x, y)
=======
        elif mode == "IMPACT_HOLD":
            set_motors(SPEED_RAM, SPEED_RAM) # Weiter drücken
            if time.time() - timer_start > 0.3:
                timer_start = time.time()
                mode = "BACKOFF"
>>>>>>> 3da9deb2544467e00c068279bedcf06b3c35a346

        elif mode == "BACKOFF":
            set_motors(SPEED_BACKOFF, SPEED_BACKOFF)
            if time.time() - timer_start > 0.5:
                stop()
                target_index += 1
                mode = "SELECT_TARGET"

        time.sleep(0.03)

except KeyboardInterrupt:
    print("Beendet.")
finally:
    stop()
    client.loop_stop()
