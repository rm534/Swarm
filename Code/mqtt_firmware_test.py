from wifi import *
from mqtt import MQTTClient
import machine
import time

def sub_cb(topic, msg):
    print(msg)

connect()

client = MQTTClient("device_id", "35.164.26.30", user="robin", password="focker12", port=1883)
client.set_callback(sub_cb)
client.connect()
client.subscribe(topic="demo.key")
while True:
    print("Sending ON")
    client.publish(topic="demo.key", msg="22.5")
    time.sleep(1)
    print("Sending OFF")
    client.publish(topic="demo.key", msg="22.1")

    time.sleep(1)
