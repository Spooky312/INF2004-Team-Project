import paho.mqtt.client as mqtt
import json

BROKER = "localhost"
TOPIC = "car/telemetry"

def on_message(client, userdata, msg):
    data = msg.payload.decode()
    try:
        j = json.loads(data)
        print(f"📡 Distance={j['distance']:.2f} cm")
    except:
        print("Raw:", data)

client = mqtt.Client()
client.on_message = on_message
client.connect(BROKER, 1883)
client.subscribe(TOPIC)
print("Listening for telemetry...")
client.loop_forever()
