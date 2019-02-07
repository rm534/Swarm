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
        Messenger.SwarmMessenger.__init__(self)
        WiFi.connect()

        return

    def init_all_network(self):
        WiFi.connect()

    def pack_state_info(self, temp, x, y, battery):
        data = pack(STATE_PACK_CODE, temp, x, y, battery)

        return data

    def send_state_wifi(self, alarm, temp, x, y, battery):
        self._send_state_wifi(temp, x, y, battery)
        logger.debug("send_state_wifi","Sending state over WiFi successfully")

    def _send_state_wifi(self, temp, x, y, battery):
        data = self.pack_state_info(temp, x, y, battery)
        self.send_prepack_msg(data, RABBITMQ_TOPIC, STATUS_MESSAGE)


if __name__ == "__main__":
    network = SwarmNetwork()
    network.send_state_wifi()
