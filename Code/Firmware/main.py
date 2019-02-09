
import Body
import pycom
import Network
from machine import Timer
import Code.Firmware.SwarmBot
import Config
import Behaviour
import Bluetooth_Comms
import Code.Firmware.Behaviour
import Code.Firmware.Bluetooth_Comms

#Test Edit

"""
import SwarmBot
import Config


VERSION = 0.0
VERSION_DATE = "Nov 2018"
Authors = ["Robin", "Sally", "James", "Ian", "Ben", "Fern", "Nick", "Billy"]
DEVICE_ID = Config.config_firmware["device"]["devid"]


def info():
    print("+------------------------+")
    print("|    SwarmBot            |")
    print("+------------------------+")
    print("| Code v{} {}            |".format(VERSION, VERSION_DATE))
    print("| Device ID: {}          |".format(DEVICE_ID))
    print("+------------------------+")


if __name__ == "__main__":
    swarmbot = SwarmBot.SwarmBot()

    try:
        swarmbot.alive()
        pass
    except:
        print("[-] Error")
        print("[-] Die Immediately")
        swarmbot.die()

"""

VERSION = 0.0
VERSION_DATE = "Nov 2018"
Authors = ["Robin", "Sally", "James", "Ian", "Ben", "Fern", "Nick", "Billy"]
DEVICE_ID = Config.config_firmware["device"]["devid"]


def info():
    print("+------------------------+")
    print("|    SwarmBot            |")
    print("+------------------------+")
    print("| Code v{} {}            |".format(VERSION, VERSION_DATE))
    print("| Device ID: {}          |".format(DEVICE_ID))
    print("+------------------------+")
"""
#Roughly what the main code should be
def current_full_main():
    #I would assumer we write the code we want in here
    #This is a basic structure of how i see main working -- Nick

    #Initialise a body object
    swarmbody = Body.SwarmBody();
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Sets initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);

    #We are now at the main while Loop

    #ALL OF THESE FUNCTIONS NEED ARGUMENTS
    while 1==1:
        #Update interal coords
        swarmbeh.Set_InternalXY():
        swarmbeh.Increment_Bounty_Tiles();
        swarmbt.Handle_Bluetooth_Behaviour();

        Current_Angle = swarmbody.get_angle();
        #THis statment needs some level of + or - degrees.
        if Current_Angle == Destination_Angle:
            swarmbeh.Robot_Movement_Behaviour();
        else:
            #Rotate the bot to get its movement angle to its destination angle
            dif = Current_Angle - Destination_Angle;
            if dif < 0:
                if dif < pi:
                    Rotate_Right();
                else:
                    Rotate_Left();
            else:
                if dif < pi:
                    Rotate_Left();
                else:
                    Rotate_Right();

        #Checks if its in a new cell and if so does a transmission
        swarmbeh.Check_New_Grid_Cell_Handle();

"""


def test1_transmit():
	#Initialise a body object
    #swarmbody = Body.SwarmBody();
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Sets initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);

    Timer = 150;
    rx = 0;
    ry = 0;
    rx2 = 0;
    ry2 = 0;
	#Currently does one of each transmission type
    while 1==1:
        if Timer == 150:
    		#Transmit A Tile Update
        	rx2 = int((uos.urandom(1)[0]/256) * 10);
        	ry2 = int((uos.urandom(1)[0]/256) * 10);
        	lum = int((uos.urandom(1)[0]/256) * 10);
        	swarmbt.Start_Transmit_Tile_Update(rx2,ry2,lum,15);
        elif Timer == 100:
    		rx = int((uos.urandom(1)[0]/256) * 10);
    		ry = int((uos.urandom(1)[0]/256) * 10);
    		#Transmit A Tile Selection
    		swarmbt.Broadcast_Tile_Selection([rx,ry],1);
        elif Timer == 50:
    		#Transmit A Tile Deselection
    		swarmbt.Broadcast_Tile_Selection([rx,ry],0);#
    	Timer-=1;

def test1_recieve():
	#Initialise a body object
    #swarmbody = Body.SwarmBody();
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Sets initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);

    while 1==1:
        swarmbt.Handle_Bluetooth_Behaviour()


if __name__ == "__main__":
    ##Swarmbot is initialised
    swarmbot = SwarmBot.SwarmBot()
    swarmbot.alive()

    print("SwarmBot is Testing -_-");
    test1_recieve();
