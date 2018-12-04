from mqtt import MQTTClient
from wifi-connection import *
import machine
import time
from config import *

class Messaging():
    def __init__(self, dev_ID, conf):
        connect()
        self.dev_ID = dev_ID
        self.conf = conf
        return

    def connect_mqtt(self):
        try:
            self.client(self.dev_ID, self.conf["server"], user=self.conf["user"], password=self.conf["pwd"], port=self.conf["port"])
            self.client.set_callback(self.sub_cb)
            self.client.connect()

        except Exception as err:
            print(err)


    def sub_cb(self, topic, msg):
        print(msg)

    def send_msg(self, topic, msg):
        self.client.publish(topic=topic, msg=msg)

if __name__ == '__main__':
    messenger = Messaging('22',config_firmware["mqtt"])
    messenger.connect_mqtt()
    messenger.send_msg("hello", "TESTING")

