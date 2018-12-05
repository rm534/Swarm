from mqtt import *
from wifi import *
import machine
import time
from config import *


class Messaging():
    def __init__(self, dev_ID, conf):
        connect()
        self.dev_ID = dev_ID
        self.conf = conf
        self.init_mqtt()
        return

    def init_mqtt(self):
        self.client = MQTTClient("device_id", "35.164.26.30", user="robin", password="focker12", port=1883)
        self.client.set_callback(self.sub_cb)
        self.client.connect()
        self.client.subscribe(topic="demo.key")

    def sub_cb(self, topic, msg):
        print(msg)

    def send_msg(self, topic, msg):
        self.client.publish(topic=topic, msg=msg)
        return


if __name__ == '__main__':
    messenger = Messaging('22', config_firmware["mqtt"])

    while True:
        print("Sending ON")
        messenger.send_msg(topic="demo.key", msg="22.5")
        time.sleep(1)
        print("Sending OFF")
        messenger.send_msg(topic="demo.key", msg="22.1")
        time.sleep(1)
