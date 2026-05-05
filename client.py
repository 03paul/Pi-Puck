import paho.mqtt.client as mqtt
import json
import time

BROKER = "192.168.178.43"
PORT = 1883

MY_ID = 38  

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
        "x": data["position"][0],
        "y": data["position"][1],
        "angle": data["angle"]
    }

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER, PORT, 60)
client.loop_start()

try:
    while True:
        pos = get_my_position()

        if pos is None:
            print("Keine Position. Sichtbare IDs:", list(robot_positions.keys()))
        else:
            print(
                f"ID={MY_ID} | "
                f"x={pos['x']:.3f}, y={pos['y']:.3f}, angle={pos['angle']:.1f}"
            )
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    client.loop_stop()
    client.disconnect()