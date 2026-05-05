import paho.mqtt.client as mqtt
import json
import time
import math
from pipuck.pipuck import PiPuck

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 38          # eigene Tracking-ID anpassen
TARGET_ID = 44      # Objekt-ID

ALIGN_TOLERANCE = 2.0
TURN_SPEED = 150
CHARGE_SPEED = 1000
CHARGE_DURATION = 2.0

robot_positions = {}

mode = "ALIGN"
charge_start_time = None


def clamp_speed(v):
    return max(-1024, min(1024, int(v)))


def normalize_angle(angle):
    return angle % 360


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def angle_to_target(robot, target):
    dx = target["x"] - robot["x"]
    dy = target["y"] - robot["y"]

    return normalize_angle(math.degrees(math.atan2(dy, dx)))


def get_state(rid):
    rid = str(rid)

    if rid not in robot_positions:
        return None

    data = robot_positions[rid]

    if "position" not in data:
        return None

    return {
        "x": data["position"][0],
        "y": data["position"][1],
        "angle": data.get("angle", None)
    }


def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe("robot_pos/all")


def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except json.JSONDecodeError:
        print("Invalid JSON")


pipuck = PiPuck(epuck_version=2)


def set_motors(left, right):
    pipuck.epuck.set_motor_speeds(
        clamp_speed(left),
        clamp_speed(right)
    )


def stop():
    set_motors(0, 0)


def turn_left():
    set_motors(-TURN_SPEED, TURN_SPEED)


def turn_right():
    set_motors(TURN_SPEED, -TURN_SPEED)


def charge_forward():
    set_motors(CHARGE_SPEED, CHARGE_SPEED)


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    while True:
        robot = get_state(MY_ID)
        target = get_state(TARGET_ID)

        if robot is None:
            print("Keine eigene Position. Sichtbare IDs:", list(robot_positions.keys()))
            stop()
            time.sleep(0.2)
            continue

        if target is None:
            print(f"Target {TARGET_ID} nicht mehr sichtbar / vom Feld. Stop.")
            stop()
            break

        if robot["angle"] is None:
            print("Kein Winkel für Roboter verfügbar.")
            stop()
            time.sleep(0.2)
            continue

        current_angle = robot["angle"]
        target_angle = angle_to_target(robot, target)
        diff = angle_diff(target_angle, current_angle)

        if mode == "ALIGN":
            if abs(diff) <= ALIGN_TOLERANCE:
                stop()
                time.sleep(0.1)

                print(
                    f"Aligned: angle={current_angle:.1f}, "
                    f"target_angle={target_angle:.1f}, diff={diff:.1f}. "
                    f"Charge for {CHARGE_DURATION}s."
                )

                charge_start_time = time.time()
                mode = "CHARGE"

            else:
                # Falls er falsch herum dreht: turn_left/turn_right tauschen
                if diff > 0:
                    turn_right()
                    turn_dir = "RIGHT"
                else:
                    turn_left()
                    turn_dir = "LEFT"

                print(
                    f"mode=ALIGN | robot=({robot['x']:.2f},{robot['y']:.2f}) "
                    f"target=({target['x']:.2f},{target['y']:.2f}) "
                    f"angle={current_angle:.1f}, target_angle={target_angle:.1f}, "
                    f"diff={diff:.1f}, turning={turn_dir}"
                )

        elif mode == "CHARGE":
            charge_forward()

            elapsed = time.time() - charge_start_time

            print(
                f"mode=CHARGE | {elapsed:.2f}/{CHARGE_DURATION:.2f}s | "
                f"robot=({robot['x']:.2f},{robot['y']:.2f}) "
                f"target=({target['x']:.2f},{target['y']:.2f})"
            )

            if elapsed >= CHARGE_DURATION:
                stop()
                time.sleep(0.1)
                mode = "ALIGN"

        time.sleep(0.03)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop()
    client.loop_stop()
    client.disconnect()