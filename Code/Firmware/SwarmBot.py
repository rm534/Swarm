import Body
import Network
import Behaviour
import machine
import slogger
import mpu6050
import sys
import VL53L0X
from ustruct import pack
from machine import Timer

class SwarmBot(Behaviour.SwarmBehaviour):
    def __init__(self):
        #slogger.basic_config(level=slogger.NOTSET)
        #self.logger = slogger.get_logger("NETWORK")
        #Behaviour.SwarmBehaviour.__init__(self)
        1==1;
    def alive(self):
        self.logger.debug("alive","Waking Up...")
        # get initial position
        # generate blank maps
        # choose target square
        # move towards destination
        # if arrived -> choose target...
        self.logger.debug("alive","Let me get my wake up routine done")
        self.wake_up_routine()
        self.logger.debug("alive", "Let me plan what to do")
        self.plan_to_do()

    def wake_up_routine(self):
        # Holds all the code for the initialisation of the bot
        # Gets GRUMPY if not done properly!!!!
        self.init_all_network()
        self.init_all_messenger()
        self.init_mqtt()
        self.logger.debug("wake_up_routine","DONE")

    def plan_to_do(self):
        # Holds all alarms that need to periodically interrupt the woken up bot
        # Forgets what to do otherwise...
        self.state_alarm = Timer.alarm(self.send_state_wifi, 300, periodic=True)
        self.logger.debug("plan_to_do","DONE")

    def kill_alarms(self):
        #Cancel tasks
        self.state_alarm.cancel()

    def die(self):
        # Done something wrong
        # Kill it off...
        pass

    def test(self):
        pass
