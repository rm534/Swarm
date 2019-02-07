import Body 
import Network 
from machine import Timer
import slogger

slogger.basic_config(level=slogger.NOTSET)
logger = slogger.get_logger("NETWORK")



def test(alarm, body=Body.SwarmBody, network=Network.SwarmNetwork):
	logger.debug("test","acquiring and sending state information")
	temp, x, y, battery = body.get_state_info()
	network.send_state_wifi(temp, x, y, battery)
	logger.debug("test","sent state information successfully")

if __name__ == "__main__":

	logger.debug("__main__", "setting up body and network")
    body = Body.SwarmBody()
    network = Network.SwarmNetwork()

    logger.debug("__main__", "initialise_gyro_new")
    body.initialise_gyro_new()
    logger.debug("__main__", "setting alarm to call position function")
    body.set_alarm_pos_test(time=3)
    logger.debug("__main__", "setting alarm to test ingestor every 10 sec")
    alarm_ingestor_test = Timer.Alarm(test,10, periodic=True, arg=(body=body, network=network))
  	

    

