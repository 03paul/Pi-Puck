import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

# ── Konfiguration ──────────────────────────────────────────────
BROKER = "192.168.178.43"
PORT   = 1883
MY_ID  = 38

# Spielfeld
X_MIN, X_MAX = 0.0, 2.0
Y_MIN, Y_MAX = 0.0, 1.0
BORDER_MARGIN = 0.12

# Geschwindigkeiten (PiPuck: -1024 … 1024)
SPEED_DRIVE = 500
SPEED_TURN  = 400

# Regler
STEER_GAIN      = 3.5
MAX_CORRECTION  = 250
AIM_THRESHOLD   = 8.0   # Grad – Toleranz beim Ausrichten
POS_TOL         = 0.08  # Meter – Toleranz Zielpunkt erreicht

# Winkel-Vorzeichen anpassen falls nötig
TURN_SIGN    =  1   # -1 falls er falsch herum dreht
STEERING_SIGN = 1   # -1 falls er beim Fahren weglenkt
ANGLE_OFFSET  = 0.0 # falls Winkel systematisch falsch

# Ecken des Rand-Pfads (Uhrzeigersinn)
CORNERS = [
    (X_MAX - BORDER_MARGIN, Y_MAX - BORDER_MARGIN),  # oben-rechts
    (X_MIN + BORDER_MARGIN, Y_MAX - BORDER_MARGIN),  # oben-links
    (X_MIN + BORDER_MARGIN, Y_MIN + BORDER_MARGIN),  # unten-links
    (X_MAX - BORDER_MARGIN, Y_MIN + BORDER_MARGIN),  # unten-rechts
]
# ──────────────────────────────────────────────────────────────

robot_positions = {}

# ── PiPuck ─────────────────────────────────────────────────────
pipuck = PiPuck(epuck_version=2)

def clamp(v):
    return max(-1024, min(1024, int(v)))

def set_motors(left, right):
    pipuck.epuck.set_motor_speeds(clamp(left), clamp(right))

def stop():
    set_motors(0, 0)

# ── MQTT ───────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe("robot_pos/all")

def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        print("Invalid JSON")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()

def get_my_position():
    rid = str(MY_ID)
    if rid not in robot_positions:
        return None
    d = robot_positions[rid]
    return {
        "x":     d["position"][0],
        "y":     d["position"][1],
        "angle": (d["angle"] + ANGLE_OFFSET) % 360
    }

# ── Geometrie-Helfer ───────────────────────────────────────────
def normalize(a):
    return a % 360

def angle_diff(target, current):
    return (target - current + 180) % 360 - 180

def angle_to_point(pos, tx, ty):
    dx = tx - pos["x"]
    dy = ty - pos["y"]
    return normalize(math.degrees(math.atan2(dy, dx)))

def dist_to_point(pos, tx, ty):
    return math.hypot(tx - pos["x"], ty - pos["y"])

def nearest_border_point(pos):
    """Nächster Punkt auf dem Rand."""
    x, y = pos["x"], pos["y"]
    d_left   = x - X_MIN
    d_right  = X_MAX - x
    d_bottom = y - Y_MIN
    d_top    = Y_MAX - y
    m = BORDER_MARGIN
    min_d = min(d_left, d_right, d_bottom, d_top)
    if min_d == d_left:   return (X_MIN + m, y)
    if min_d == d_right:  return (X_MAX - m, y)
    if min_d == d_bottom: return (x, Y_MIN + m)
    return                       (x, Y_MAX - m)

def is_on_border(pos):
    return (
        pos["x"] <= X_MIN + BORDER_MARGIN or
        pos["x"] >= X_MAX - BORDER_MARGIN or
        pos["y"] <= Y_MIN + BORDER_MARGIN or
        pos["y"] >= Y_MAX - BORDER_MARGIN
    )

# ── Fahr-Funktionen ────────────────────────────────────────────
def turn_towards(pos, tx, ty):
    """Dreht auf der Stelle Richtung (tx, ty). Gibt True zurück wenn fertig."""
    target_angle = angle_to_point(pos, tx, ty)
    diff = angle_diff(target_angle, pos["angle"])

    if abs(diff) <= AIM_THRESHOLD:
        stop()
        return True

    if TURN_SIGN * diff > 0:
        set_motors( SPEED_TURN, -SPEED_TURN)
    else:
        set_motors(-SPEED_TURN,  SPEED_TURN)

    return False

def drive_towards(pos, tx, ty):
    """Fährt mit Lenkkorrektur auf (tx, ty). Gibt True zurück wenn Ziel erreicht."""
    if dist_to_point(pos, tx, ty) < POS_TOL:
        return True

    target_angle = angle_to_point(pos, tx, ty)
    diff = angle_diff(target_angle, pos["angle"])

    correction = STEERING_SIGN * STEER_GAIN * diff
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    left  = max(120, SPEED_DRIVE - correction)
    right = max(120, SPEED_DRIVE + correction)

    set_motors(left, right)
    return False

# ── State Machine ──────────────────────────────────────────────
phase        = "GOTO_BORDER"
corner_index = 0
aimed        = False   # True sobald wir auf die aktuelle Ecke ausgerichtet sind

print("Warte auf Positionsdaten...")
while not robot_positions:
    time.sleep(0.1)
print("Los!")

try:
    while True:
        pos = get_my_position()

        if pos is None:
            print("Keine Position. Sichtbare IDs:", list(robot_positions.keys()))
            stop()
            time.sleep(0.2)
            continue

        # ── Phase 1: Zum Rand fahren ───────────────────────────
        if phase == "GOTO_BORDER":
            if is_on_border(pos):
                print("Rand erreicht → FOLLOW_BORDER")
                phase  = "FOLLOW_BORDER"
                aimed  = False
                # Starte bei der nächsten Ecke
                corner_index = min(
                    range(len(CORNERS)),
                    key=lambda i: math.hypot(
                        CORNERS[i][0] - pos["x"],
                        CORNERS[i][1] - pos["y"]
                    )
                )
            else:
                tx, ty = nearest_border_point(pos)
                # Erst ausrichten, dann fahren
                if not aimed:
                    aimed = turn_towards(pos, tx, ty)
                else:
                    arrived = drive_towards(pos, tx, ty)
                    if arrived:
                        aimed = False
                print(
                    f"GOTO_BORDER | pos=({pos['x']:.2f},{pos['y']:.2f}) "
                    f"angle={pos['angle']:.1f}° → Rand ({tx:.2f},{ty:.2f})"
                )

        # ── Phase 2: Am Rand entlang ───────────────────────────
        elif phase == "FOLLOW_BORDER":
            tx, ty = CORNERS[corner_index]
            d = dist_to_point(pos, tx, ty)

            # Ecke erreicht → nächste Ecke
            if d < POS_TOL * 1.5:
                corner_index = (corner_index + 1) % len(CORNERS)
                aimed = False
                print(f"Ecke erreicht → nächste Ecke {corner_index}: {CORNERS[corner_index]}")
            else:
                if not aimed:
                    aimed = turn_towards(pos, tx, ty)
                else:
                    drive_towards(pos, tx, ty)

            print(
                f"FOLLOW_BORDER | pos=({pos['x']:.2f},{pos['y']:.2f}) "
                f"angle={pos['angle']:.1f}° → Ecke {corner_index} ({tx:.2f},{ty:.2f}) d={d:.3f}"
            )

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    stop()
    client.loop_stop()
    client.disconnect()