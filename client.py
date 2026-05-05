import paho.mqtt.client as mqtt
import json
import time
import math

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 39          # eigene Roboter-ID anpassen
TARGET_ID = 44      # Objekt-ID

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

def get_state(rid):
    rid = str(rid)

    if rid not in robot_positions:
        return None

    data = robot_positions[rid]

    return {
        "x": data["position"][0],
        "y": data["position"][1],
        "angle": data.get("angle", None)
    }

def normalize_angle(angle):
    return angle % 360

def angle_diff(target, current):
    """
    Differenz zwischen Zielwinkel und aktuellem Winkel.
    Ergebnis: -180 bis +180 Grad.
    """
    return (target - current + 180) % 360 - 180

def angle_to_target(robot, target):
    dx = target["x"] - robot["x"]
    dy = target["y"] - robot["y"]

    target_angle = math.degrees(math.atan2(dy, dx))
    return normalize_angle(target_angle)

def distance(robot, target):
    dx = target["x"] - robot["x"]
    dy = target["y"] - robot["y"]

    return math.sqrt(dx * dx + dy * dy)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    while True:
        robot = get_state(MY_ID)
        obj = get_state(TARGET_ID)

        if robot is None:
            print("Keine Roboterposition. Sichtbare IDs:", list(robot_positions.keys()))
            time.sleep(0.5)
            continue

        if obj is None:
            print("Objekt 44 nicht sichtbar. Sichtbare IDs:", list(robot_positions.keys()))
            time.sleep(0.5)
            continue

        target_angle = angle_to_target(robot, obj)
        dist = distance(robot, obj)

        if robot["angle"] is not None:
            diff = angle_diff(target_angle, robot["angle"])

            print(
                f"Robot {MY_ID}: x={robot['x']:.3f}, y={robot['y']:.3f}, angle={robot['angle']:.1f}° | "
                f"Object {TARGET_ID}: x={obj['x']:.3f}, y={obj['y']:.3f} | "
                f"target_angle={target_angle:.1f}°, diff={diff:.1f}°, distance={dist:.3f}m"
            )

        else:
            print(
                f"Robot {MY_ID}: x={robot['x']:.3f}, y={robot['y']:.3f} | "
                f"Object {TARGET_ID}: x={obj['x']:.3f}, y={obj['y']:.3f} | "
                f"target_angle={target_angle:.1f}°, distance={dist:.3f}m"
            )

        time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    client.loop_stop()
    client.disconnect()