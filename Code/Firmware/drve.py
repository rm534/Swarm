
#
## Micro-Python Script to send a swarm robot
## to a specific coordinate.
## Open - loop method based on estimated motor speed alone.
## Also records current coordinate / location to nearest 1cm.
#

## Libraries ##
import pycom

import time
from machine import Timer

from math import atan
from math import sin
from math import cos
from math import pi

from machine import PWM

from machine import Pin


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




DCv = 0.6;
pycom.heartbeat(False)
#Initialise a body object
swarmbody = Body.SwarmBody();
swarmbody.battery = 100;
#Initalise a bluetooth controller
swarmbt = Bluetooth_Comms.SwarmBluetooth();
#Initialise a behaviour controller
#swarmbeh = Behaviour.SwarmBehaviour();
#Choose an initial destination
#swarmbeh.Choose_Target_Square(swarmbt,swarmbody);

#swarmbody.initialise_lidar(swarmbody.SDA, swarmbody.SCL, swarmbody.lidar_DIO1, swarmbody.lidar_DIO2, swarmbody.lidar_DIO3, swarmbody.lidar_DIO4)
X = 0;
Y = 0;
Xg = 1000;
Yg = 1000;
#print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
lastcol = True;
ldtmer = 0;
mml = 150;
while True:
    #print(str(X)+"/"+str(Y));

    #swarmbeh.Set_InternalXY(X,Y);
    #swarmbeh.Increment_Bounty_Tiles(1);
    #swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
    #swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(swarmbody,swarmbt);
    #Xg = swarmbeh.Target_Destination[0]*swarmbeh.Arena_Grid_Size_X;
    #Yg = swarmbeh.Target_Destination[1]*swarmbeh.Arena_Grid_Size_Y;
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
            forward();
            print("forward")
        lastcol = False;
        #Green light
        pycom.rgbled(0x007f00)
        #forward();
        #print("forward")
