from mqtt import *
from Config import *
from ustruct import pack
import utime
import machine
import Config
import slogger as logging

HEADER_PACK_FORMAT = ">QhLhh"
STATUS_MESSAGE = 0x01


class SwarmMessenger():
    def __init__(self):
        self.init_all_messenger()
        self.init_mqtt()

        return

    def init_all_messenger(self):

        self.dev_ID = Config.config_firmware["device"]["devid"]
        self.rtc = machine.RTC()

    def init_mqtt(self):
        self.client = MQTTClient(self.dev_ID, "34.221.207.211", user="robin", password="focker12", port=1883)
        self.client.set_callback(self.sub_cb)
        self.client.connect()
        self.client.subscribe(topic="demo.key")

    def sub_cb(self, topic, msg):
        print(msg)

    def make_header(self, message_type):
        now = self.rtc.now()
        ts = utime.mktime(now)

        ms = int(now[6])
        mid = machine.rng()

        return pack(HEADER_PACK_FORMAT, ts, ms, mid, int(self.dev_ID), message_type)

    def send_prepack_msg(self, msg, topic, message_type):
        header = self.make_header(message_type=message_type)
        self.client.publish(topic=topic, msg=header + msg)

    def send_msg(self, topic, msg):
        self.client.publish(topic=topic, msg=msg)
        return


if __name__ == '__main__':
    messenger = SwarmMessenger('22', config_firmware["mqtt"])

    while True:
        pass
