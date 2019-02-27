import Messenger
import WiFi
import Config
import utime
from ustruct import pack
from Body import *
import slogger
from machine import Timer

RABBITMQ_TOPIC = "demo.key"
STATE_PACK_CODE = ">ffff"
STATUS_MESSAGE = 0x01
slogger.basic_config(level=slogger.NOTSET)
logger = slogger.get_logger("NETWORK")


class SwarmNetwork(Messenger.SwarmMessenger):
    def __init__(self):
        WiFi.connect()
        Messenger.SwarmMessenger.__init__(self)


        return

    def init_all_network(self):
        WiFi.connect()

    def pack_state_info(self, temp, x, y, battery):
        data = pack(STATE_PACK_CODE, temp, x, y, battery)

        return data

    def send_state_wifi(self, alarm):
        self._send_state_wifi(temp=22, x=2.3, y=3.2, battery=3.3)
        print("hello")

    def _send_state_wifi(self, temp, x, y, battery):
        data = self.pack_state_info(temp, x, y, battery)
        self.send_prepack_msg(data, RABBITMQ_TOPIC, STATUS_MESSAGE)
        logger.debug("_send_state_", "sent state successfully")

    def _alarm_send_state(self, alarm):
        self.send_state_wifi()

    def alarm_send_state(self, time):
        alarm = Timer.Alarm(self._alarm_send_state, time=100, periodic=True)
        return alarm


if __name__ == "__main__":
    network = SwarmNetwork()
    network.send_state_wifi()
