
from network import Bluetooth
import Body
import pycom
import Network
from machine import Timer
#import Code.Firmware.SwarmBot
import Config
import Behaviour
import Bluetooth_Comms
import SwarmBot
import uos

import time
from machine import Timer

from math import atan
from math import pi
from machine import PWM

from math import sin
from math import cos

from machine import Pin
#import Code.Firmware.Behaviour
#import Code.Firmware.Bluetooth_Comms

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
    swarmbody = Body.SwarmBody();
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Sets initial destination


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
    		#swarmbt.Broadcast_Tile_Selection([rx,ry],1);
        elif Timer == 50:
    		#Transmit A Tile Deselection
    	    #swarmbt.Broadcast_Tile_Selection([rx,ry],1);
            1==1;
        elif Timer == 0:
            Timer = 32000;
    	Timer-=1;

def test1_recieve():
	#Initialise a body object
    swarmbody = Body.SwarmBody();
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Sets initial destination

    while 1==1:
        swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,True);

def test0_transmit():
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    while 1==1:
        swarmbt.Broadcast_Tile_Selection([2,2],0);



def transmit_basic():
    abluetooth = Bluetooth()

    abluetooth.set_advertisement(name="a", manufacturer_data="l", service_data="99999")
    abluetooth.advertise(True)
    while True:
        1==1;


#To select a square then simulate moving towards it while makling transmission the entire timer
def test2_both():
	#Initialise a body object
    swarmbody = Body.SwarmBody();
    swarmbody.battery = 100;
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
    X = 0;
    Y = 0;
    print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
    while True:
        #print(str(X)+"/"+str(Y));
        swarmbeh.Set_InternalXY(X,Y);
        swarmbeh.Increment_Bounty_Tiles(1);
        swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
        swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(swarmbody,swarmbt);
        Xg = swarmbeh.Target_Destination[0]*swarmbeh.Arena_Grid_Size_X;
        Yg = swarmbeh.Target_Destination[1]*swarmbeh.Arena_Grid_Size_Y;
        #This movement is scuffed it will go diagonal until one coord is met but this is for testing purposes only !
        if X < Xg:
            X += 0.5;
        else:
            X -= 0.5;
        if Y < Yg:
            Y += 0.5;
        else:
            Y -= 0.5;


#Have the light on the pycom turn red if pycoms are too close and green if they are far enough apartself.
#We will be using code from test 2
def test3_both():
    pycom.heartbeat(False)
	#Initialise a body object
    swarmbody = Body.SwarmBody();
    swarmbody.battery = 100;
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
    X = 0;
    Y = 0;
    print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
    while True:
        #print(str(X)+"/"+str(Y));

        swarmbeh.Set_InternalXY(X,Y);
        swarmbeh.Increment_Bounty_Tiles(1);
        swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
        swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(swarmbody,swarmbt);
        Xg = swarmbeh.Target_Destination[0]*swarmbeh.Arena_Grid_Size_X;
        Yg = swarmbeh.Target_Destination[1]*swarmbeh.Arena_Grid_Size_Y;
        #This movement is scuffed it will go diagonal until one coord is met but this is for testing purposes only !
        if X < Xg:
            X += 0.5;
        else:
            X -= 0.5;
        if Y < Yg:
            Y += 0.5;
        else:
            Y -= 0.5;






        if swarmbt.Collision_Timer > 0:
            #red Light
            pycom.rgbled(0x7f0000)
            #print(swarmbt.Collision_Timer);
        else:
            #Green light
            pycom.rgbled(0x007f00)




def test2_monitor():
    1==1;


## Setup Timer ##

chrono = Timer.Chrono()

## Speed Parameters ##
DCw = 0.7
DCv = 1

w = 200
v = 0.425

## Starting Coordinates ##
org = (0, 0)
ang_org = 0


## Set Pycom Heartbeat ##
pycom.heartbeat(True)


## Motor Setups ##
#Motor 1
Motor1F = Pin('P11', mode =Pin.OUT)
Motor1B = Pin('P12', mode =Pin.OUT)
# pwm.channel(0, pin ='P5', duty_cycle = DC)

#Motor 2
Motor2F = Pin('P21', mode =Pin.OUT)
Motor2B = Pin('P20', mode =Pin.OUT)
PWM2 = 'P7'
# pwm.channel(0, pin ='P7', duty_cycle = DC)

# Standby Pin always on
#Pin('P22', mode =Pin.OUT).value(1)

## PWM ##
pwm = PWM(0, frequency = 500)

## Functions ##
def stop_all():
    Motor1F.value(0)
    Motor2F.value(0)
    Motor1B.value(0)
    Motor2B.value(0)

    # PWM
    Pin('P6', mode =Pin.OUT).value(0)
    Pin('P7', mode =Pin.OUT).value(0)



## Movement with Timer ##
def forward():
    chrono.start()
    Motor1F.value(1)
    Motor2F.value(1)

    pwm.channel(0, pin ='P6', duty_cycle = DCv)
    pwm.channel(0, pin ='P7', duty_cycle = DCv)


def back():
    chrono.start()

    Motor1B.value(1)
    Motor2B.value(1)

    pwm.channel(0, pin ='P6', duty_cycle = DCv)
    pwm.channel(0, pin ='P7', duty_cycle = DCv)


def clockwise():
    chrono.start()
    Motor1B.value(1)
    Motor2F.value(1)

    pwm.channel(0, pin ='P6', duty_cycle = DCw)
    pwm.channel(0, pin ='P7', duty_cycle = DCw)


def anti_clockwise():
    chrono.start()
    Motor1F.value(1)
    Motor2B.value(1)

    pwm.channel(0, pin ='P6', duty_cycle = DCw)
    pwm.channel(0, pin ='P7', duty_cycle = DCw)




def commands():
    print("Enter Coordinates for robot to move to.")
    print("Enter Current Coordinates to return to angle 0.")


def best_route(COMM, org, ang_org):
    """
    Plans most efficient route from one coordinate to another.

    Returns:
    1. Rotational Tuple
       0 == Anticlockwise
       1 == Clockwise
       Angle to move

    2. Distance Tuple
       0 == Anticlockwise
       1 == Clockwise
       Distance to move

    3. Robot's absolute angle after movement.
    """

    deltaX = (COMM[0] - org[0])
    deltaY = (COMM[1] - org[1])

    dist = ((deltaX)**2 + (deltaY)**2)**0.5

    if deltaY == 0:
        if deltaX == 0:
            ang_desired = 0

        elif deltaX > 0:
            ang_desired = 90

        elif deltaX < 0:
            ang_desired = 270


    elif deltaX == 0:
        if deltaY > 0:
            ang_desired = 0

        elif deltaY < 0:
            ang_desired = 180


    else:
        if deltaX > 0 and deltaY > 0:
            n = 0

        elif deltaX > 0 and deltaY < 0:
            n = -180

        elif deltaX < 0 and deltaY < 0:
            n = 180

        elif deltaX < 0 and deltaY > 0:
            n = -360

        ang_desired = abs(abs(atan(deltaX / deltaY) * (180/pi)) + n)


    ang_mov = ang_desired - ang_org


    if ang_desired < ang_org:
        ang_mov = ang_mov + 360

    else:
        ang_mov = abs(ang_mov)


    if ang_mov <= 90:

        rot = 1
        direct = 1

        # Clockwise
        # Forwards


    elif ang_mov > 90 and ang_mov <= 180:

        if ang_desired >= 180 and ang_desired < 360:
            ang_desired = ang_desired - 180

        elif ang_desired < 180 and ang_desired >= 0:
            ang_desired = ang_desired + 180

        ang_mov = abs(180 - ang_mov)

        rot = 0
        direct = 0

        # Anticlockwise
        # Backwards


    elif ang_mov > 180 and ang_mov <= 270:

        if ang_desired >= 180 and ang_desired < 360:
            ang_desired = ang_desired - 180

        elif ang_desired < 180 and ang_desired >= 0:
            ang_desired = ang_desired + 180

        ang_mov = abs(180 - ang_mov)

        rot = 1
        direct = 0

        # Clockwise
        # Backwards


    elif ang_mov > 270:

        ang_mov = 360 - ang_mov

        rot = 0
        direct = 1

        # Anticlockwise
        # Forwards

    return (rot, ang_mov), (direct, dist), ang_desired

def ang_mov_through(lin_mov, ang_desired):
    """
    Calculates the acute angle from the robots absolute angle.
    So it's position between two coordinates can be calculated.

    Returns floating point number.
    """
    # Converts absolute angle if robot is reversing.
    if lin_mov[0] == 1:
        pass

    elif lin_mov[0] == 0:
        if ang_desired < 180 and ang_desired >= 0:
            ang_desired = (ang_desired + 180)

        elif ang_desired >= 180 and ang_desired < 360:
            ang_desired = (ang_desired - 180)


    if ang_desired <= 90 and ang_desired >= 0:
        return ang_desired

    elif ang_desired < 180 and ang_desired > 90:
        return (90 - (ang_desired - 90))

    elif ang_desired < 270 and ang_desired >= 180:
        return (ang_desired - 180)

    elif ang_desired <= 360 and ang_desired >= 270:
        return (90 - (ang_desired - 270))


def current_loc(COMM, org, ang_mov_thoug):
    """
    Returns current X and Y coordinates as a tuple
    to the nearest 1cm.
    """

    deltaX = (COMM[0] - org[0])
    deltaY = (COMM[1] - org[1])


    c_time = chrono.read()

    if deltaX >= 0:
        c_x_loc = round(v*c_time*sin(ang_mov_thoug*(pi/180)) + org[0], 2)


    elif deltaX <= 0:
        c_x_loc = round(-v*c_time*sin(ang_mov_thoug*(pi/180)) + org[0], 2)


    if deltaY >= 0:
        c_y_loc = round(v*c_time*cos(ang_mov_thoug*(pi/180)) + org[1], 2)


    elif deltaY <= 0:
        c_y_loc = round(-v*c_time*cos(ang_mov_thoug*(pi/180)) + org[1], 2)

    return (c_x_loc, c_y_loc)

#Colson testng
def test4_collson():
        DCv = 0.6;
        pycom.heartbeat(False)
    	#Initialise a body object
        swarmbody = Body.SwarmBody();
        swarmbody.battery = 100;
        #Initalise a bluetooth controller
        swarmbt = Bluetooth_Comms.SwarmBluetooth();
        #Initialise a behaviour controller
        swarmbeh = Behaviour.SwarmBehaviour();
        #Choose an initial destination
        swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
        X = 0;
        Y = 0;
        print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
        lastcol = True;
        while True:
            #print(str(X)+"/"+str(Y));

            swarmbeh.Set_InternalXY(X,Y);
            swarmbeh.Increment_Bounty_Tiles(1);
            swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
            swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(swarmbody,swarmbt);
            Xg = swarmbeh.Target_Destination[0]*swarmbeh.Arena_Grid_Size_X;
            Yg = swarmbeh.Target_Destination[1]*swarmbeh.Arena_Grid_Size_Y;
            #This movement is scuffed it will go diagonal until one coord is met but this is for testing purposes only !

            if X < Xg:
                X += 0.5;
            else:
                X -= 0.5;
            if Y < Yg:
                Y += 0.5;
            else:
                Y -= 0.5;
                #stop_all();
            if swarmbt.Collision_Timer > 0:
                if lastcol == False:
                    stop_all();
                    back();
                    print("back")
                #red Light
                lastcol = True;
                pycom.rgbled(0x7f0000)
                print(swarmbt.Collision_Timer);
                #back();

            else:
                if lastcol == True:
                    stop_all();
                    lastcol = False;
                    forward();
                    print("forward")
                lastcol = False;
                #Green light
                pycom.rgbled(0x007f00)
                #forward();
                #print("forward")




def test5_ldar():


    DCv = 0.6;
    pycom.heartbeat(False)
	#Initialise a body object
    swarmbody = Body.SwarmBody();
    swarmbody.battery = 100;
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);

    #swarmbody.initialise_lidar(swarmbody.SDA, swarmbody.SCL, swarmbody.lidar_DIO1, swarmbody.lidar_DIO2, swarmbody.lidar_DIO3, swarmbody.lidar_DIO4)
    X = 0;
    Y = 0;
    print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
    lastcol = True;
    ldtmer = 0;
    mml = 150;
    while True:
        #print(str(X)+"/"+str(Y));

        swarmbeh.Set_InternalXY(X,Y);
        swarmbeh.Increment_Bounty_Tiles(1);
        swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
        swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(swarmbody,swarmbt);
        Xg = swarmbeh.Target_Destination[0]*swarmbeh.Arena_Grid_Size_X;
        Yg = swarmbeh.Target_Destination[1]*swarmbeh.Arena_Grid_Size_Y;
        #This movement is scuffed it will go diagonal until one coord is met but this is for testing purposes only !

        if X < Xg:
            X += 0.5;
        else:
            X -= 0.5;
        if Y < Yg:
            Y += 0.5;
        else:
            Y -= 0.5;
            #stop_all();
        l1, l2, l3, l4 = swarmbody.get_lidar();



        if l1 < mml or l2 < mml or l3 < mml or l4 < mml:
            ldtmer = 10;

        if ldtmer > 0:
            if lastcol == False:
                stop_all();
                back();
                print("back")
            #red Light
            lastcol = True;
            pycom.rgbled(0x7f0000)
            print(ldtmer);
            #back();
            ldtmer-=1;
        else:
            if lastcol == True:
                stop_all();
                lastcol = False;
                #forward();
                print("forward")
            lastcol = False;
            #Green light
            pycom.rgbled(0x007f00)
            #forward();
            #print("forward")

    #Makng movement based on read coords
def test6_movement():
    1==1;
    DCv = 0.6;
    pycom.heartbeat(False)
	#Initialise a body object
    swarmbody = Body.SwarmBody();
    swarmbody.battery = 100;
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
    X = 0;
    Y = 0;
    print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
    lastcol = True;
    while True:
        #print(str(X)+"/"+str(Y));

        swarmbeh.Set_InternalXY(X,Y);
        swarmbeh.Increment_Bounty_Tiles(1);
        swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
        swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(swarmbody,swarmbt);
        Xg = swarmbeh.Target_Destination[0]*swarmbeh.Arena_Grid_Size_X;
        Yg = swarmbeh.Target_Destination[1]*swarmbeh.Arena_Grid_Size_Y;

        #Rotate to face drecrton

        #Turn on motor untl dest reached



        if X < Xg:
            X += 0.5;
        else:
            X -= 0.5;
        if Y < Yg:
            Y += 0.5;
        else:
            Y -= 0.5;
        stop_all();
        if swarmbt.Collision_Timer > 0:
            if lastcol == False:
                stop_all();
                #back();
                print("back")
            #red Light
            lastcol = True;
            pycom.rgbled(0x7f0000)
            print(swarmbt.Collision_Timer);
            #back();

        else:
            if lastcol == True:
                stop_all();
                lastcol = False;
                #forward();
                print("forward")
            lastcol = False;
            #Green light
            pycom.rgbled(0x007f00)
            #forward();
            #print("forward")


def test7_both_int():

    ## Set Pycom Heartbeat ##
    pycom.heartbeat(True)


    ## Setup Timer ##
    chrono = Timer.Chrono()


    ## Starting Coordinates ##
    org = (0, 0)
    ang_org = 0


    ## Sample Rate [S] ##
    sr = 0.1


    ##  Initial Speed Parameters ##
    # PWM DC
    DCw = 0.7
    DCv = 0.3

    # Speed
    w = 300  # [Deg/s]
    v = 0.35 # [m/s]


    ## Motor Setups ##
    # Motor 1
    Motor1F = Pin('P11', mode =Pin.OUT)
    Motor1B = Pin('P12', mode =Pin.OUT)
    PWM1 = 'P6'
    # PWM on 'P6'

    # Motor 2
    Motor2F = Pin('P21', mode =Pin.OUT)
    Motor2B = Pin('P20', mode =Pin.OUT)
    PWM2 = 'P7'
    # PWM on'P7'


    ## PWM Setup ##
    pwm = PWM(0, frequency = 500)

    ## Functions ##
    def stop_all():
        Motor1F.value(0)
        Motor2F.value(0)
        Motor1B.value(0)
        Motor2B.value(0)

        # Stop PWM
        Pin(PWM1, mode = Pin.OUT).value(0)
        Pin(PWM2, mode = Pin.OUT).value(0)



    ## Defining Functions ##
    def forward():
        chrono.start()
        Motor1F.value(1)
        Motor2F.value(1)
        Motor1B.value(0)
        Motor2B.value(0)

        pwm.channel(0, pin = PWM1, duty_cycle = DCv)
        pwm.channel(0, pin = PWM2, duty_cycle = DCv)


    def back():
        chrono.start()

        Motor1B.value(1)
        Motor2B.value(1)
        Motor1F.value(0)
        Motor2F.value(0)


        pwm.channel(0, pin = PWM1, duty_cycle = DCv)
        pwm.channel(0, pin = PWM2, duty_cycle = DCv)


    def clockwise():
        chrono.start()
        Motor1B.value(1)
        Motor2F.value(1)
        Motor1F.value(0)
        Motor2B.value(0)

        pwm.channel(0, pin = PWM1, duty_cycle = DCw)
        pwm.channel(0, pin = PWM2, duty_cycle = DCw)


    def anti_clockwise():
        chrono.start()
        Motor1F.value(1)
        Motor2B.value(1)
        Motor1B.value(0)
        Motor2F.value(0)

        pwm.channel(0, pin = PWM1, duty_cycle = DCw)
        pwm.channel(0, pin = PWM2, duty_cycle = DCw)


    def commands():
        print("Enter Coordinates for robot to move to.")
        print("Enter Current Coordinates to return to angle 0.")



    def best_route(COMM, org, ang_org):
        """
        Plans most efficient route from one coordinate to another.

        Returns:
        1. Rotational Tuple
           0 == Anticlockwise
           1 == Clockwise
           Angle to move

        2. Distance Tuple
           0 == Anticlockwise
           1 == Clockwise
           Distance to move

        3. Robot's absolute angle after movement.
        """

        deltaX = (COMM[0] - org[0])
        deltaY = (COMM[1] - org[1])

        dist = ((deltaX)**2 + (deltaY)**2)**0.5

        if deltaY == 0:
            if deltaX == 0:
                ang_desired = 0

            elif deltaX > 0:
                ang_desired = 90

            elif deltaX < 0:
                ang_desired = 270


        elif deltaX == 0:
            if deltaY > 0:
                ang_desired = 0

            elif deltaY < 0:
                ang_desired = 180


        else:
            if deltaX > 0 and deltaY > 0:
                n = 0

            elif deltaX > 0 and deltaY < 0:
                n = -180

            elif deltaX < 0 and deltaY < 0:
                n = 180

            elif deltaX < 0 and deltaY > 0:
                n = -360

            ang_desired = abs(abs(atan(deltaX / deltaY) * (180/pi)) + n)


        ang_mov = ang_desired - ang_org


        if ang_desired < ang_org:
            ang_mov = ang_mov + 360

        else:
            ang_mov = abs(ang_mov)


        if ang_mov <= 90:

            rot = 1
            direct = 1

            # Clockwise
            # Forwards


        elif ang_mov > 90 and ang_mov <= 180:

            if ang_desired >= 180 and ang_desired < 360:
                ang_desired = ang_desired - 180

            elif ang_desired < 180 and ang_desired >= 0:
                ang_desired = ang_desired + 180

            ang_mov = abs(180 - ang_mov)

            rot = 0
            direct = 0

            # Anticlockwise
            # Backwards


        elif ang_mov > 180 and ang_mov <= 270:

            if ang_desired >= 180 and ang_desired < 360:
                ang_desired = ang_desired - 180

            elif ang_desired < 180 and ang_desired >= 0:
                ang_desired = ang_desired + 180

            ang_mov = abs(180 - ang_mov)

            rot = 1
            direct = 0

            # Clockwise
            # Backwards


        elif ang_mov > 270:

            ang_mov = 360 - ang_mov

            rot = 0
            direct = 1

            # Anticlockwise
            # Forwards

        return (rot, ang_mov), (direct, dist), ang_desired


    def ang_mov_through(lin_mov, ang_desired):
        """
        Calculates the acute angle from the robots absolute angle.
        So it's position between two coordinates can be calculated.

        Returns floating point number.
        """
        # Converts absolute angle if robot is reversing.
        if lin_mov[0] == 1:
            pass

        elif lin_mov[0] == 0:
            if ang_desired < 180 and ang_desired >= 0:
                ang_desired = (ang_desired + 180)

            elif ang_desired >= 180 and ang_desired < 360:
                ang_desired = (ang_desired - 180)


        if ang_desired <= 90 and ang_desired >= 0:
            return ang_desired

        elif ang_desired < 180 and ang_desired > 90:
            return (90 - (ang_desired - 90))

        elif ang_desired < 270 and ang_desired >= 180:
            return (ang_desired - 180)

        elif ang_desired <= 360 and ang_desired >= 270:
            return (90 - (ang_desired - 270))


    def current_loc(COMM, org, ang_mov_thoug):
        """
        Returns current X and Y coordinates as a tuple
        to the nearest 1cm.
        """

        deltaX = (COMM[0] - org[0])
        deltaY = (COMM[1] - org[1])


        c_time = chrono.read()

        if deltaX >= 0:
            c_x_loc = round(v*c_time*sin(ang_mov_thoug*(pi/180)) + org[0], 2)


        elif deltaX <= 0:
            c_x_loc = round(-v*c_time*sin(ang_mov_thoug*(pi/180)) + org[0], 2)


        if deltaY >= 0:
            c_y_loc = round(v*c_time*cos(ang_mov_thoug*(pi/180)) + org[1], 2)


        elif deltaY <= 0:
            c_y_loc = round(-v*c_time*cos(ang_mov_thoug*(pi/180)) + org[1], 2)

        return (c_x_loc, c_y_loc)

    def rotate_to_angle(Xg,Yg):
        COMM = (float(Xg), float(Yg))

        result = best_route(COMM, org, ang_org)

        ang_desired = result[2]


        ## Rotational Movement ##
        rot_mov = result[0]

        t_rot = rot_mov[1] / w
        #print(t_rot)
        if rot_mov[0] == 1:
            clockwise()

        elif rot_mov[0] == 0:

            anti_clockwise()

        return t_rot;

    def move_to_cord(Xg,Yg):
        COMM = (float(Xg), float(Yg))

        result = best_route(COMM, org, ang_org)
        ang_desired = result[2]

        lin_mov = result[1]
        print(lin_mov);

        t_lin = lin_mov[1] / v

        ang_mov_thoug = ang_mov_through(lin_mov, ang_desired)

        if lin_mov[0] == 1:
            forward()

        elif lin_mov[0] == 0:
            back()

        return t_lin;
    #does a 360 for calbraton
    def do_a_360():
        t_rot = 360 / w;
        chrono.stop();
        chrono.reset();
        clockwise();
        while chrono.read() < t_rot:
            #keep rollng
            pass

        stop_all();

    #do_a_360();

    pycom.heartbeat(False)
	#Initialise a body object
    swarmbody = Body.SwarmBody();
    swarmbody.battery = 100;
    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
    X = 0;
    Y = 0;
    print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
    fw = False;
    bk = False;
    ck = False;
    ack = False;
    r_flag = False;
    m_flag = False;
    Xg = 0;
    Yg = 0;
    swarmbeh.Map_Bounty[0][8] = 5000;
    swarmbeh.Map_Bounty[8][8] = 5000;
    swarmbeh.Map_Bounty[8][0] = 5000;
    j = 0;
    while True:
        j+=1;
        print(j)
        swarmbeh.Set_InternalXY(X,Y);
        #swarmbeh.Increment_Bounty_Tiles(1);


        swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
        swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(swarmbody,swarmbt);
        old_xg = Xg;
        old_yg = Yg;
        Xg = swarmbeh.Target_Destination[0]*swarmbeh.Arena_Grid_Size_X/3000;
        Yg = swarmbeh.Target_Destination[1]*swarmbeh.Arena_Grid_Size_Y/3000;
        #This movement is scuffed it will go diagonal until one coord is met but this is for testing purposes only !
        if swarmbt.Collision_Timer > 0:
            #red Light
            pycom.rgbled(0x7f0000)
            #print(swarmbt.Collision_Timer);
        else:
            #Green light
            pycom.rgbled(0x007f00)



        #F we are gong to a new coord
        if Xg != old_xg or Yg != old_yg:
            if m_flag == True:
                print("Stop drvng")
                chrono.stop()
                chrono.reset()
                #time.sleep(1)  # Delay for stability.
                m_flag = False;
                org = COMM
                ang_org = ang_desired



            COMM = (float(Xg), float(Yg))
            result = best_route(COMM, org, ang_org)
            ang_desired = result[2]
            lin_mov = result[1]
            ang_mov_thoug = ang_mov_through(lin_mov, ang_desired)
            print("Start Rotato")
            r_flag = True;

            ang_t = rotate_to_angle(Xg,Yg);
        #f we are at our dest
        #THS MAY BE GARBAGE
        elif abs(X-Xg) < 0.001 and abs(Y-Yg) < 0.001:
            #CHoose a new locaton
            swarmbeh.Choose_Target_Square(swarmbt,swarmbody)



        if r_flag == True:
            if chrono.read() > ang_t:
                print("Stp rotate")
                stop_all()
                chrono.stop()
                chrono.reset()
                #time.sleep(1)  # Delay for stability.
                drv_t = move_to_cord(Xg,Yg);
                m_flag = True;
                r_flag = False;

        if m_flag == True:
            if chrono.read() > drv_t:
                print("Stop drvng")
                chrono.stop()
                chrono.reset()
                time.sleep(0.001)  # Delay for stability.
                m_flag = False;
                org = COMM
                ang_org = ang_desired
            else:
                print(COMM)
                #print(org)
                #print(ang_mov_thoug)
                #print(chrono.read())
                c_loc = current_loc(COMM, org, ang_mov_thoug)
                #update internal coords
                X = c_loc[0]*3000;
                Y = c_loc[1]*3000;
                print("Robot's current Position is:", c_loc)




def test8_moveput():

    #
    ## Micro-Python Script to send a swarm robot
    ## to a specific coordinate.
    ## Open - loop method based on estimated motor speed alone.
    ## Also records current coordinate / location to nearest 1cm.
    #

    ## Libraries ##



    ## Set Pycom Heartbeat ##
    pycom.heartbeat(True)


    ## Setup Timer ##
    chrono = Timer.Chrono()


    ## Starting Coordinates ##
    org = (0, 0)
    ang_org = 0


    ## Sample Rate [S] ##
    sr = 0.1


    ##  Initial Speed Parameters ##
    # PWM DC
    DCw = 0.7
    DCv = 1

    # Speed
    w = 200   # [Deg/s]
    v = 0.425 # [m/s]


    ## Motor Setups ##
    # Motor 1
    Motor1F = Pin('P11', mode =Pin.OUT)
    Motor1B = Pin('P12', mode =Pin.OUT)
    PWM1 = 'P6'
    # PWM on 'P6'

    # Motor 2
    Motor2F = Pin('P21', mode =Pin.OUT)
    Motor2B = Pin('P20', mode =Pin.OUT)
    PWM2 = 'P7'
    # PWM on'P7'


    ## PWM Setup ##
    pwm = PWM(0, frequency = 500)

    ## Functions ##
    def stop_all():
        Motor1F.value(0)
        Motor2F.value(0)
        Motor1B.value(0)
        Motor2B.value(0)

        # Stop PWM
        Pin(PWM1, mode = Pin.OUT).value(0)
        Pin(PWM2, mode = Pin.OUT).value(0)



    ## Defining Functions ##
    def forward():
        chrono.start()
        Motor1F.value(1)
        Motor2F.value(1)

        pwm.channel(0, pin = PWM1, duty_cycle = DCv)
        pwm.channel(0, pin = PWM2, duty_cycle = DCv)


    def back():
        chrono.start()

        Motor1B.value(1)
        Motor2B.value(1)

        pwm.channel(0, pin = PWM1, duty_cycle = DCv)
        pwm.channel(0, pin = PWM2, duty_cycle = DCv)


    def clockwise():
        chrono.start()
        Motor1B.value(1)
        Motor2F.value(1)

        pwm.channel(0, pin = PWM1, duty_cycle = DCw)
        pwm.channel(0, pin = PWM2, duty_cycle = DCw)


    def anti_clockwise():
        chrono.start()
        Motor1F.value(1)
        Motor2B.value(1)

        pwm.channel(0, pin = PWM1, duty_cycle = DCw)
        pwm.channel(0, pin = PWM2, duty_cycle = DCw)


    def commands():
        print("Enter Coordinates for robot to move to.")
        print("Enter Current Coordinates to return to angle 0.")



    def best_route(COMM, org, ang_org):
        """
        Plans most efficient route from one coordinate to another.

        Returns:
        1. Rotational Tuple
           0 == Anticlockwise
           1 == Clockwise
           Angle to move

        2. Distance Tuple
           0 == Anticlockwise
           1 == Clockwise
           Distance to move

        3. Robot's absolute angle after movement.
        """

        deltaX = (COMM[0] - org[0])
        deltaY = (COMM[1] - org[1])

        dist = ((deltaX)**2 + (deltaY)**2)**0.5

        if deltaY == 0:
            if deltaX == 0:
                ang_desired = 0

            elif deltaX > 0:
                ang_desired = 90

            elif deltaX < 0:
                ang_desired = 270


        elif deltaX == 0:
            if deltaY > 0:
                ang_desired = 0

            elif deltaY < 0:
                ang_desired = 180


        else:
            if deltaX > 0 and deltaY > 0:
                n = 0

            elif deltaX > 0 and deltaY < 0:
                n = -180

            elif deltaX < 0 and deltaY < 0:
                n = 180

            elif deltaX < 0 and deltaY > 0:
                n = -360

            ang_desired = abs(abs(atan(deltaX / deltaY) * (180/pi)) + n)


        ang_mov = ang_desired - ang_org


        if ang_desired < ang_org:
            ang_mov = ang_mov + 360

        else:
            ang_mov = abs(ang_mov)


        if ang_mov <= 90:

            rot = 1
            direct = 1

            # Clockwise
            # Forwards


        elif ang_mov > 90 and ang_mov <= 180:

            if ang_desired >= 180 and ang_desired < 360:
                ang_desired = ang_desired - 180

            elif ang_desired < 180 and ang_desired >= 0:
                ang_desired = ang_desired + 180

            ang_mov = abs(180 - ang_mov)

            rot = 0
            direct = 0

            # Anticlockwise
            # Backwards


        elif ang_mov > 180 and ang_mov <= 270:

            if ang_desired >= 180 and ang_desired < 360:
                ang_desired = ang_desired - 180

            elif ang_desired < 180 and ang_desired >= 0:
                ang_desired = ang_desired + 180

            ang_mov = abs(180 - ang_mov)

            rot = 1
            direct = 0

            # Clockwise
            # Backwards


        elif ang_mov > 270:

            ang_mov = 360 - ang_mov

            rot = 0
            direct = 1

            # Anticlockwise
            # Forwards

        return (rot, ang_mov), (direct, dist), ang_desired


    def ang_mov_through(lin_mov, ang_desired):
        """
        Calculates the acute angle from the robots absolute angle.
        So it's position between two coordinates can be calculated.

        Returns floating point number.
        """
        # Converts absolute angle if robot is reversing.
        if lin_mov[0] == 1:
            pass

        elif lin_mov[0] == 0:
            if ang_desired < 180 and ang_desired >= 0:
                ang_desired = (ang_desired + 180)

            elif ang_desired >= 180 and ang_desired < 360:
                ang_desired = (ang_desired - 180)


        if ang_desired <= 90 and ang_desired >= 0:
            return ang_desired

        elif ang_desired < 180 and ang_desired > 90:
            return (90 - (ang_desired - 90))

        elif ang_desired < 270 and ang_desired >= 180:
            return (ang_desired - 180)

        elif ang_desired <= 360 and ang_desired >= 270:
            return (90 - (ang_desired - 270))


    def current_loc(COMM, org, ang_mov_thoug):
        """
        Returns current X and Y coordinates as a tuple
        to the nearest 1cm.
        """

        deltaX = (COMM[0] - org[0])
        deltaY = (COMM[1] - org[1])


        c_time = chrono.read()

        if deltaX >= 0:
            c_x_loc = round(v*c_time*sin(ang_mov_thoug*(pi/180)) + org[0], 2)


        elif deltaX <= 0:
            c_x_loc = round(-v*c_time*sin(ang_mov_thoug*(pi/180)) + org[0], 2)


        if deltaY >= 0:
            c_y_loc = round(v*c_time*cos(ang_mov_thoug*(pi/180)) + org[1], 2)


        elif deltaY <= 0:
            c_y_loc = round(-v*c_time*cos(ang_mov_thoug*(pi/180)) + org[1], 2)

        return (c_x_loc, c_y_loc)


    ## Main Loop ##
    while True:
        X = input('\n'"Enter X - Coordinate Here: ")
        Y = input("Enter Y - Coordinate Here: ")

        if X == "position" or Y == "position":
            print('\n'"Current Coordinate: ", org)
            print("Current Angle: ", ang_org)
            continue

        elif X == "command" or Y == "command":
            commands()
            continue

        elif X == "" or Y == "":
            continue


        COMM = (float(X), float(Y))

        result = best_route(COMM, org, ang_org)

        ang_desired = result[2]


        ## Rotational Movement ##
        rot_mov = result[0]

        t_rot = rot_mov[1] / w

        if rot_mov[0] == 1:
            clockwise()

        elif rot_mov[0] == 0:
            anti_clockwise()

        while (chrono.read() < t_rot):
            # Does nothing while rotating.
            pass

        stop_all()
        chrono.stop()
        chrono.reset()

        time.sleep(1)  # Delay for stability.

        ## Linear Movement ##
        lin_mov = result[1]

        t_lin = lin_mov[1] / v

        ang_mov_thoug = ang_mov_through(lin_mov, ang_desired)

        if lin_mov[0] == 1:
            forward()

        elif lin_mov[0] == 0:
            back()

        while (chrono.read() < t_lin):
            c_loc = current_loc(COMM, org, ang_mov_thoug)
            print("Robot's current Position is:", c_loc)
            time.sleep(sr)
            pass

        stop_all()
        chrono.stop()
        chrono.reset()

        ## Printing Movement Statistics ##
        print('\n'"Orginal Coordinate: ", org)
        print("New Coordinate: ", COMM)

        print('\n'"Orignal Angle: ", ang_org)
        print("New Angle: ", ang_desired)

        print('\n'"Distance Moved: ", lin_mov[1])
        print("Angle Moved Through: ", rot_mov[1])

        print('\n'"Angles [Deg]")
        print("Distances [m]")

        ## Updates New Position and Absolute Angle ##
        org = COMM
        ang_org = ang_desired

if __name__ == "__main__":
    ##Swarmbot is initialised
    #swarmbot = SwarmBot.SwarmBot()
    #swarmbot.alive()
    #swarmbt = Bluetooth_Comms.SwarmBluetooth();

    #swarmbeh = Behaviour.SwarmBehaviour();
    print("SwarmBot is Testing -_-");
    test7_both_int();
