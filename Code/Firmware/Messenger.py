from mqtt import *
from Config import *
import utime
from machine import RTC, rng
from ustruct import pack
import Config

HEADER_PACK_FORMAT = ">QhLhh"
STATUS_MESSAGE = 0x01
logging.basic_config(level=logging.NOTSET)
logger = logging.get_logger("MESSENGER")

class Messenger():
    def __init__(self, dev_ID, conf):
        self.dev_ID = dev_ID
        self.conf = conf
        self.init_mqtt()
        self.rtc = RTC()
        self.dev_ID = Config.config_firmware["device"]["devid"]
        return

    def init_mqtt(self):
        self.client = MQTTClient("device_id", "35.164.26.30", user="robin", password="focker12", port=1883)
        self.client.set_callback(self.sub_cb)
        self.client.connect()
        self.client.subscribe(topic="demo.key")

    def sub_cb(self, topic, msg):
        print(msg)

    def make_header(self, message_type):
        ts = utime.mktime()
        now = self.rtc.now()
        ms = int(now[6])
        mid = rng()

        return pack(HEADER_PACK_FORMAT, ts, ms, mid, self.dev_ID, message_type)

    def send_prepack_msg(self, msg, topic, message_type):
        header = self.make_header(message_type)
        self.client.publish(topic=topic, msg=header+msg)

    def send_msg(self, topic, msg):
        self.client.publish(topic=topic, msg=msg)
        return




if __name__ == '__main__':
    messenger = Messenger('22', config_firmware["mqtt"])

    while True:
        pass