import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

# ── Konfiguration ──────────────────────────────────────────────
BROKER = "192.168.178.43"
PORT   = 1883
MY_ID  = 38

X_MIN, X_MAX = 0.0, 2.0
Y_MIN, Y_MAX = 0.0, 1.0
BORDER_MARGIN = 0.12

SPEED_DRIVE = 500
SPEED_TURN  = 400

STEER_GAIN     = 3.5
MAX_CORRECTION = 250
AIM_THRESHOLD  = 10.0  # Grad – etwas großzügiger
POS_TOL        = 0.08  # Meter

# ── Winkel-Kalibrierung ────────────────────────────────────────
# Teste: Roboter zeigt physisch nach Osten (rechts) → was zeigt angle?
# Trage diesen Wert hier ein:
ANGLE_NORTH_OFFSET = 90.0  # 90 = Roboter nutzt 0°=Norden (Standard)
#                            0  = Roboter nutzt 0°=Osten
# Dreht er falsch herum beim Ausrichten:
TURN_SIGN = 1   # auf -1 setzen
# ──────────────────────────────────────────────────────────────

robot_positions = {}
pipuck = PiPuck(epuck_version=2)

def clamp(v):
    return max(-1024, min(1024, int(v)))

def set_motors(left, right):
    pipuck.epuck.set_motor_speeds(clamp(left), clamp(right))

def stop():
    set_motors(0, 0)

def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe("robot_pos/all")

def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        pass

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
        "angle": d["angle"] % 360
    }

def angle_diff(target, current):
    """Kürzeste Differenz, positiv = rechts drehen."""
    return (target - current + 180) % 360 - 180

def angle_to_point(pos, tx, ty):
    """
    Zielwinkel im gleichen Koordinatensystem wie pos['angle'].
    ANGLE_NORTH_OFFSET=90 → 0°=Norden, CW (typisch für Tracking-Systeme)
    """
    dx = tx - pos["x"]
    dy = ty - pos["y"]
    # atan2 standard: 0°=Osten, CCW
    deg_east = math.degrees(math.atan2(dy, dx))
    # Umrechnen auf Roboter-Koordinatensystem
    return (deg_east + ANGLE_NORTH_OFFSET) % 360

def dist_to(pos, tx, ty):
    return math.hypot(tx - pos["x"], ty - pos["y"])

def nearest_border_point(pos):
    x, y = pos["x"], pos["y"]
    m = BORDER_MARGIN
    d_left   = x - X_MIN
    d_right  = X_MAX - x
    d_bottom = y - Y_MIN
    d_top    = Y_MAX - y
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

CORNERS = [
    (X_MAX - BORDER_MARGIN, Y_MAX - BORDER_MARGIN),
    (X_MIN + BORDER_MARGIN, Y_MAX - BORDER_MARGIN),
    (X_MIN + BORDER_MARGIN, Y_MIN + BORDER_MARGIN),
    (X_MAX - BORDER_MARGIN, Y_MIN + BORDER_MARGIN),
]

# ── State Machine ──────────────────────────────────────────────
phase        = "GOTO_BORDER"
corner_index = 0
state        = "TURN"   # "TURN" oder "DRIVE"
tx, ty       = 0.0, 0.0

def compute_target(pos):
    global tx, ty
    if phase == "GOTO_BORDER":
        tx, ty = nearest_border_point(pos)
    else:
        tx, ty = CORNERS[corner_index]

# Warte auf Position
print("Warte auf Positionsdaten...")
while True:
    pos = get_my_position()
    if pos:
        break
    time.sleep(0.1)

compute_target(pos)
target_angle = angle_to_point(pos, tx, ty)
print(f"Start! pos=({pos['x']:.2f},{pos['y']:.2f}) angle={pos['angle']:.1f}°")
print(f"Ziel: ({tx:.2f},{ty:.2f}) → target_angle={target_angle:.1f}°")

try:
    while True:
        pos = get_my_position()
        if pos is None:
            stop()
            time.sleep(0.2)
            continue

        d = dist_to(pos, tx, ty)
        target_angle = angle_to_point(pos, tx, ty)
        diff = angle_diff(target_angle, pos["angle"])

        # ── Ziel erreicht ──────────────────────────────────────
# ── Ziel erreicht ──────────────────────────────────────
        if d < POS_TOL:
            stop()
            print(f"Ziel ({tx:.2f},{ty:.2f}) erreicht!")

            if phase == "GOTO_BORDER":
                # WECHSEL ERZWINGEN: Wenn wir nah genug am berechneten Randpunkt sind, 
                # wechseln wir in den Follow-Modus.
                print("→ Rand-Punkt erreicht, starte FOLLOW_BORDER")
                phase = "FOLLOW_BORDER"
                
                # Finde die nächste Ecke, um den Rundlauf zu starten
                corner_index = min(
                    range(len(CORNERS)),
                    key=lambda i: math.hypot(
                        CORNERS[i][0] - pos["x"],
                        CORNERS[i][1] - pos["y"]
                    )
                )
            else:
                # In der FOLLOW_BORDER Phase: Einfach zur nächsten Ecke springen
                corner_index = (corner_index + 1) % len(CORNERS)
                print(f"→ Nächste Ecke: {corner_index} = {CORNERS[corner_index]}")

            # Neues Ziel basierend auf der neuen Phase/Ecke berechnen
            compute_target(pos)
            state = "TURN" # Zuerst wieder ausrichten
            print(f"Neues Ziel: ({tx:.2f},{ty:.2f}), state=TURN")
            time.sleep(0.2) # Kurz warten für Stabilität
            continue

        # ── TURN: Ausrichten ───────────────────────────────────
        if state == "TURN":
            print(
                f"[TURN] phase={phase} | "
                f"pos=({pos['x']:.2f},{pos['y']:.2f}) angle={pos['angle']:.1f}° "
                f"→ ({tx:.2f},{ty:.2f}) target={target_angle:.1f}° diff={diff:+.1f}°"
            )

            if abs(diff) <= AIM_THRESHOLD:
                stop()
                time.sleep(0.1)  # kurze Pause damit Motoren stoppen
                state = "DRIVE"
                print("→ Ausgerichtet, starte DRIVE")
            else:
                # diff > 0 = Ziel ist rechts → linkes Rad schneller
                if TURN_SIGN * diff > 0:
                    set_motors( SPEED_TURN, -SPEED_TURN)   # dreht rechts
                else:
                    set_motors(-SPEED_TURN,  SPEED_TURN)   # dreht links

        # ── DRIVE: Geradeaus mit Korrektur ─────────────────────
        elif state == "DRIVE":
            # Zu stark abgewichen → neu ausrichten
            if abs(diff) > 40:
                stop()
                state = "TURN"
                print(f"→ Kurskorrektur (diff={diff:+.1f}°), zurück zu TURN")
            else:
                correction = STEER_GAIN * diff
                correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))
                left  = max(150, SPEED_DRIVE - correction)
                right = max(150, SPEED_DRIVE + correction)
                set_motors(left, right)

                print(
                    f"[DRIVE] phase={phase} | "
                    f"pos=({pos['x']:.2f},{pos['y']:.2f}) angle={pos['angle']:.1f}° "
                    f"→ ({tx:.2f},{ty:.2f}) d={d:.3f} diff={diff:+.1f}° "
                    f"L={int(left)} R={int(right)}"
                )

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    stop()
    client.loop_stop()
    client.disconnect()