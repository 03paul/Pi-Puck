import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

Broker = "192.168.178.43"
Port = 1883

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print(data)
    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)
client.loop_start()

# PiPuck init
pipuck = PiPuck(epuck_version=2)

# Parameter (feinjustieren!)
FORWARD_SPEED = 500      # beide Räder gleich → geradeaus
MOVE_DURATION = 0.5      # Sekunden fahren
STOP_DURATION = 0.5      # Sekunden Pause

try:
    for _ in range(100):  # Anzahl Schritte

        # 🔹 Vorwärts fahren
        pipuck.epuck.set_motor_speeds(FORWARD_SPEED, FORWARD_SPEED)
        time.sleep(MOVE_DURATION)

        # 🔹 Stoppen
        pipuck.epuck.set_motor_speeds(0, 0)
        time.sleep(STOP_DURATION)

finally:
    # Safety Stop
    pipuck.epuck.set_motor_speeds(0, 0)
    client.loop_stop()