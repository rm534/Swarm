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
        return

    def connect_mqtt(self):
        try:
            print(self.dev_ID, self.conf["server"], self.conf["user"], self.conf["pwd"])
            self.client = MQTTClient(client_id='Robin',server=self.conf["server"], user="robin", password="focker12", port=1883, ssl=True)
            self.client.set_callback(None)
            #self.client.subscribe(topic='demo.key')
            self.client.connect(clean_session=True)

        except Exception as err:
            print("Error:",err)

    def disconnect_mqtt(self):
        self.client.disconnect()

    def sub_cb(self, topic, msg):
        print(msg)

    def send_msg(self, topic, msg):
        print(self.client.connection_info())
        self.client.publish(topic=topic, msg=msg)

if __name__ == '__main__':
    messenger = Messaging('22',config_firmware["mqtt"])
    messenger.connect_mqtt()
    messenger.send_msg("demo.key", "TESTING")
    messenger.disconnect_mqtt()
