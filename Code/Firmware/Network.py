import Messenger
import WiFi
import Config
import utime
import ustruct
from Body import *
from Network import *
import slogger as logging

RABBITMQ_TOPIC = "demo.key"
STATE_PACK_CODE = ">ffff"
STATUS_MESSAGE = 0x01
logging.basic_config(level=logging.NOTSET)
logger = logging.get_logger("NETWORK")


class SwarmNetwork():
    def __init__(self):
        WiFi.connect()
        self.messenger = Messenger.Messenger("22", Config.config_firmware["mqtt"])
        logger.debug("__init__", "Network setup complete")

        return

    def get_state_info(self):
        # Get information, Dummy Variables for now

        temp = 0.7
        x = 2.3
        y = 3.2
        battery = 2.2
        logger.debug("get_state_info", "state acquired | temp: {} x: {} y: {} battery: {}".format(temp, x, y, battery))
        return temp, x, y, battery

    def pack_state_info(self, temp, x, y, battery):
        data = ustruct.pack(STATE_PACK_CODE, temp, x, y, battery)
        logger.debug("pack_state_info", "state info packed successfully")
        return data

    def send_state_wifi(self):
        temp, x, y, battery = self.get_state_info()
        data = self.pack_state_info(temp, x, y, battery)
        try:
            self.messenger.send_prepack_msg(data, RABBITMQ_TOPIC, STATUS_MESSAGE)
            logger.debug("send_state_wifi", "sent state sucessfully")
            return True
        except:
            logger.debug("send_state_wifi", "send failed")

            return False
