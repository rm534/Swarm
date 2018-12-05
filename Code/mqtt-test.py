import paho.mqtt.client as paho
import time

mqtthost = "35.164.26.30"
mqttuser = "robin"
mqttpass = "focker12"
mqtttopic = "demo.key"

def on_connect(client, userdata, flags, rc):
    print("CONNACK received with code %d." % (rc))
def on_publish(client, userdata, mid):
    print("mid: "+str(mid))

client = paho.Client()
client.on_connect = on_connect
client.on_publish = on_publish
client.username_pw_set(mqttuser,mqttpass)
client.connect(mqtthost, 1883,60)


client.loop_start()

while True:
    message = "test data"
    (rc, mid) = client.publish(mqtttopic, str(message), qos=1)
    time.sleep(10)
