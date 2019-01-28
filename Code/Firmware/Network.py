import Messenger
import WiFi
import Config
import utime
import ustruct

RABBITMQ_TOPIC = "demo.key"
STATE_PACK_CODE = "fffiiiiiiiif"

class SwarmNetwork():
    def __init__(self):
        WiFi.connect()
        self.messenger = Messenger.Messenger("22", Config.config_firmware["mqtt"])

        return

    def get_state_info(self):
        #Get information, Dummy Variables for now

        temp = 0.7
        x = 2.3
        y = 3.2
        _time = utime.gmtime()
        battery = 2.2
        return temp, x, y, _time, battery

    def pack_state_info(self, temp, x, y, _time, battery):
        data = ustruct.pack(STATE_PACK_CODE, temp, x, y, _time[0], _time[1], _time[2], _time[3], _time[4], _time[5], _time[6], _time[7], battery)
        return data

    def send_state_wifi(self, state):
        self.messenger.send_msg(topic=RABBITMQ_TOPIC, msg=state)

