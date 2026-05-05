import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

# ── Konfiguration ──────────────────────────────────────────────
BROKER = "192.168.178.43"
PORT   = 1883
MY_ID  = "39"
TARGET_ID = "44"

# Spielfeld-Grenzen
X_MIN, X_MAX = 0.0, 2.0
Y_MIN, Y_MAX = 0.0, 1.0

# Parameter für den Angriff
ATTACK_OFFSET = 0.25  # Distanz hinter dem Obstacle (in Metern)
STRIKE_ZONE   = 0.10  # Ab dieser Nähe zum Attack-Point wird "geschoben"
MAX_SPEED     = 600
STEER_GAIN    = 4.5   # Wie aggressiv er den Kurs korrigiert

robot_positions = {}
pipuck = PiPuck(epuck_version=2)

# ── Hilfsfunktionen ────────────────────────────────────────────
def clamp(v): return max(-1000, min(1000, int(v)))

def set_motors(l, r): pipuck.epuck.set_motor_speeds(clamp(l), clamp(r))

def get_pos(rid):
    d = robot_positions.get(str(rid))
    if not d: return None
    return {"x": d["position"][0], "y": d["position"][1], "angle": d["angle"] % 360}

def on_message(c, u, msg):
    global robot_positions
    try: robot_positions = json.loads(msg.payload.decode())
    except: pass

# ── Vektor-Mathematik ──────────────────────────────────────────
def get_nearest_exit(target_pos):
    """Berechnet den Punkt auf der Grenze, der dem Obstacle am nächsten ist."""
    tx, ty = target_pos["x"], target_pos["y"]
    dist_to_edge = [
        (tx - X_MIN, (X_MIN - 0.2, ty)), # West
        (X_MAX - tx, (X_MAX + 0.2, ty)), # Ost
        (ty - Y_MIN, (tx, Y_MIN - 0.2)), # Süd
        (Y_MAX - ty, (tx, Y_MAX + 0.2))  # Nord
    ]
    return min(dist_to_edge, key=lambda x: x[0])[1]

def get_attack_vector(target_pos, exit_point):
    """Berechnet den Punkt HINTER dem Obstacle."""
    dx = target_pos["x"] - exit_point[0]
    dy = target_pos["y"] - exit_point[1]
    mag = math.hypot(dx, dy)
    # Einheitsvektor von Exit zu Target, dann skaliert um OFFSET
    ax = target_pos["x"] + (dx / mag) * ATTACK_OFFSET
    ay = target_pos["y"] + (dy / mag) * ATTACK_OFFSET
    return ax, ay

# ── MQTT Setup ─────────────────────────────────────────────────
client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.subscribe("robot_pos/all")
client.loop_start()

print("Suche Obstacle 44...")

try:
    while True:
        me = get_pos(MY_ID)
        target = get_pos(TARGET_ID)

        if not me or not target:
            set_motors(0, 0)
            time.sleep(0.1)
            continue

        # 1. Wo muss das Obstacle hin?
        exit_pt = get_nearest_exit(target)
        # 2. Wo muss ich mich aufstellen?
        attack_pt = get_attack_vector(target, exit_pt)
        
        # Distanz-Checks
        dist_to_attack = math.hypot(attack_pt[0] - me["x"], attack_pt[1] - me["y"])
        dist_to_target = math.hypot(target["x"] - me["x"], target["y"] - me["y"])

        # 3. Zielwahl: Wenn ich am Attack-Point bin, ramme das Obstacle direkt zum Exit
        if dist_to_attack < STRIKE_ZONE:
            goal_x, goal_y = exit_pt[0], exit_pt[1]
            speed = MAX_SPEED
            mode = "RAMMING"
        else:
            goal_x, goal_y = attack_pt
            speed = 400
            mode = "POSITIONING"

        # 4. Dynamische Kurskorrektur (P-Regler)
        target_angle_rad = math.atan2(goal_y - me["y"], goal_x - me["x"])
        # Umrechnung auf dein 90°-Offset-System (Norden=0)
        target_angle_deg = (math.degrees(target_angle_rad) + 90) % 360
        
        diff = (target_angle_deg - me["angle"] + 180) % 360 - 180
        
        # Wenn der Winkel extrem falsch ist, erst auf der Stelle drehen
        if abs(diff) > 60:
            turn = 300 if diff > 0 else -300
            set_motors(turn, -turn)
        else:
            # Während der Fahrt korrigieren
            correction = diff * STEER_GAIN
            set_motors(speed - correction, speed + correction)

        print(f"Mode: {mode} | Dist to Attack: {dist_to_attack:.2f}m | Diff: {diff:.1f}°")
        time.sleep(0.05)

except KeyboardInterrupt:
    set_motors(0, 0)
    client.loop_stop()