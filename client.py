import paho.mqtt.client as mqtt
import json
import time
import math

# ── Konfiguration ──────────────────────────────────────────────
BROKER     = "192.168.178.43"
PORT       = 1883
MY_ID      = 39

# Spielfeld-Grenzen (anpassen!)
FIELD_MIN_X = 0.0
FIELD_MAX_X = 2.0
FIELD_MIN_Y = 0.0
FIELD_MAX_Y = 1.0

# Rand-Toleranz: wie nah = "am Rand"
BORDER_MARGIN  = 0.08

# Fahrparameter
BASE_SPEED     = 0.4   # Grundgeschwindigkeit (0.0 – 1.0)
TURN_GAIN      = 2.5   # Wie stark gelenkt wird
ANGLE_TOL      = 8.0   # Winkeltoleranz in Grad
POS_TOL        = 0.05  # Positionstoleranz in Metern

# MQTT Topics
CMD_TOPIC = f"robot_cmd/{MY_ID}"   # {"left": 0.4, "right": 0.4}
# ──────────────────────────────────────────────────────────────

robot_positions = {}

def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe("robot_pos/all")

def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        print("Invalid JSON")

def get_my_position():
    rid = str(MY_ID)
    if rid not in robot_positions:
        return None
    data = robot_positions[rid]
    return {
        "x":     data["position"][0],
        "y":     data["position"][1],
        "angle": data["angle"]        # 0° = Norden, clockwise
    }

def send_motors(left: float, right: float):
    """Schickt Motorbefehl per MQTT."""
    payload = json.dumps({"left": round(left, 3), "right": round(right, 3)})
    client.publish(CMD_TOPIC, payload)

def stop():
    send_motors(0, 0)

# ── Hilfs-Geometrie ────────────────────────────────────────────

def angle_to_target(pos, tx, ty):
    """Zielwinkel vom Roboter zum Punkt (tx, ty) in Grad, 0°=Norden, CW."""
    dx = tx - pos["x"]
    dy = ty - pos["y"]
    # math.atan2: 0=Osten, CCW → umrechnen auf 0=Norden, CW
    rad = math.atan2(dx, dy)          # Norden-basiert, CW positiv
    deg = math.degrees(rad) % 360
    return deg

def angle_diff(target_deg, current_deg):
    """Kürzeste Winkeldifferenz, positiv = nach rechts drehen."""
    diff = (target_deg - current_deg + 540) % 360 - 180
    return diff

def drive_toward(pos, tx, ty):
    """
    Fährt mit P-Regler auf Punkt (tx, ty) zu.
    Gibt False zurück, wenn Ziel noch nicht erreicht.
    Gibt True zurück, wenn Ziel erreicht.
    """
    dist = math.hypot(tx - pos["x"], ty - pos["y"])
    if dist < POS_TOL:
        return True  # Ziel erreicht

    target_angle = angle_to_target(pos, tx, ty)
    diff = angle_diff(target_angle, pos["angle"])

    # P-Regler: Lenkkorrektur proportional zum Winkelfehler
    steer = (diff / 180.0) * TURN_GAIN
    left  = max(-1.0, min(1.0, BASE_SPEED + steer))
    right = max(-1.0, min(1.0, BASE_SPEED - steer))
    send_motors(left, right)
    return False

def is_on_border(pos):
    """True wenn der Roboter bereits am Rand ist."""
    return (
        pos["x"] <= FIELD_MIN_X + BORDER_MARGIN or
        pos["x"] >= FIELD_MAX_X - BORDER_MARGIN or
        pos["y"] <= FIELD_MIN_Y + BORDER_MARGIN or
        pos["y"] >= FIELD_MAX_Y - BORDER_MARGIN
    )

def nearest_border_point(pos):
    """Nächster Punkt auf dem Rand."""
    x, y = pos["x"], pos["y"]
    # Abstand zu jeder Wand
    d_left   = x - FIELD_MIN_X
    d_right  = FIELD_MAX_X - x
    d_bottom = y - FIELD_MIN_Y
    d_top    = FIELD_MAX_Y - y

    min_d = min(d_left, d_right, d_bottom, d_top)

    if min_d == d_left:
        return (FIELD_MIN_X + BORDER_MARGIN, y)
    elif min_d == d_right:
        return (FIELD_MAX_X - BORDER_MARGIN, y)
    elif min_d == d_bottom:
        return (x, FIELD_MIN_Y + BORDER_MARGIN)
    else:
        return (x, FIELD_MAX_Y - BORDER_MARGIN)

def border_follow_target(pos):
    """
    Gibt den nächsten Wegpunkt am Rand zurück,
    sodass der Roboter im Uhrzeigersinn am Rand entlang fährt.
    Eckreihenfolge: (max_x, max_y) → (min_x, max_y) → (min_x, min_y) → (max_x, min_y) → ...
    """
    M = BORDER_MARGIN
    corners = [
        (FIELD_MAX_X - M, FIELD_MAX_Y - M),   # oben-rechts
        (FIELD_MIN_X + M, FIELD_MAX_Y - M),   # oben-links
        (FIELD_MIN_X + M, FIELD_MIN_Y + M),   # unten-links
        (FIELD_MAX_X - M, FIELD_MIN_Y + M),   # unten-rechts
    ]

    x, y = pos["x"], pos["y"]
    # Nächste Ecke finden
    nearest = min(corners, key=lambda c: math.hypot(c[0]-x, c[1]-y))
    idx = corners.index(nearest)

    # Wenn nah genug an der Ecke → zur nächsten
    if math.hypot(nearest[0]-x, nearest[1]-y) < POS_TOL * 1.5:
        idx = (idx + 1) % len(corners)

    return corners[idx]

# ── Hauptprogramm ──────────────────────────────────────────────

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.loop_start()

# Warte auf erste Position
print("Warte auf Positionsdaten...")
while not robot_positions:
    time.sleep(0.1)

phase = "GOTO_BORDER"   # oder "FOLLOW_BORDER"
print(f"Phase: {phase}")

try:
    while True:
        pos = get_my_position()
        if pos is None:
            print("Keine Position. Sichtbare IDs:", list(robot_positions.keys()))
            time.sleep(0.2)
            continue

        if phase == "GOTO_BORDER":
            if is_on_border(pos):
                print("Rand erreicht → Phase: FOLLOW_BORDER")
                phase = "FOLLOW_BORDER"
            else:
                tx, ty = nearest_border_point(pos)
                drive_toward(pos, tx, ty)

        elif phase == "FOLLOW_BORDER":
            tx, ty = border_follow_target(pos)
            drive_toward(pos, tx, ty)
            print(
                f"x={pos['x']:.3f} y={pos['y']:.3f} "
                f"angle={pos['angle']:.1f}° → Ziel ({tx:.2f},{ty:.2f})"
            )

        time.sleep(0.05)   # ~20 Hz

except KeyboardInterrupt:
    print("Stopping...")
finally:
    stop()
    client.loop_stop()
    client.disconnect()