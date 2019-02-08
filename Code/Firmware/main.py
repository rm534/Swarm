import Network
from machine import Timer
import SwarmBot
import Config

import Behaviour
import Bluetooth_Comms

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

if __name__ == "__main__":
    ##Swarmbot is initialised
    swarmbot = SwarmBot.SwarmBot()
    swarmbot.alive()

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
