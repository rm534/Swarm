from mqtt import *
from wifi import *
import machine
import time
from config import *
import utime
import ustruct


PACK_CODE = "fffiiiiiiiif"

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

    def get_info(self):
        #Get information, Dummy Variables for now
        temp = 0.7
        x = 2.3
        y = 3.2
        _time = utime.gmtime()
        battery = 2.2
        return temp, x, y, _time, battery

    def pack_info(self, temp, x, y, _time, battery):
        data = ustruct.pack(PACK_CODE, temp, x, y, _time[0], _time[1], _time[2], _time[3], _time[4], _time[5], _time[6], _time[7], battery)
        return data

    def send_update(self):
        data = self.pack_info(self.get_info())
        self.send_msg(topic="demo.key", msg=data)



if __name__ == '__main__':
    messenger = Messaging('22', config_firmware["mqtt"])

    while True:
        messenger.send_update()