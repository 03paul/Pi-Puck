import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

robot_positions = {}
MY_ID = 39  # ANPASSEN!

# Spielfeld (approx aus Dashboard)
X_MIN, X_MAX = 0.0, 2.0
Y_MIN, Y_MAX = 0.0, 1.5
THRESHOLD = 0.15  # Abstand zum Rand

def on_connect(client, userdata, flags, rc):
    print("Connected:", rc)
    client.subscribe("robot_pos/all")

def on_message(client, userdata, msg):
    global robot_positions
    try:
        robot_positions = json.loads(msg.payload.decode())
    except:
        pass

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(Broker, Port, 60)
client.loop_start()

pipuck = PiPuck(epuck_version=2)

def get_my_pos():
    if str(MY_ID) in robot_positions:
        return robot_positions[str(MY_ID)]["pos"]
    return None

def move_forward():
    pipuck.epuck.set_motor_speeds(500, 500)

def turn_left():
    pipuck.epuck.set_motor_speeds(-300, 300)

def turn_right():
    pipuck.epuck.set_motor_speeds(300, -300)

def stop():
    pipuck.epuck.set_motor_speeds(0, 0)

try:
    while True:

        pos = get_my_pos()

        if not pos:
            print("No position yet")
            stop()
            time.sleep(0.2)
            continue

        x, y = pos
        print(f"x={x:.2f}, y={y:.2f}")

        # 🔴 Check ob am Rand
        near_left   = abs(x - X_MIN) < THRESHOLD
        near_right  = abs(x - X_MAX) < THRESHOLD
        near_bottom = abs(y - Y_MIN) < THRESHOLD
        near_top    = abs(y - Y_MAX) < THRESHOLD

        # 🟢 Verhalten am Rand
        if near_bottom:
            # entlang unten → nach rechts
            move_forward()

        elif near_right:
            # entlang rechts → nach oben
            turn_left()

        elif near_top:
            # entlang oben → nach links
            turn_left()

        elif near_left:
            # entlang links → nach unten
            turn_left()

        else:
            # 🔵 Noch nicht am Rand → geh nach unten links
            turn_right()

        time.sleep(0.2)

finally:
    stop()
    client.loop_stop()