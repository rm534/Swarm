
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

import _thread
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

#Similar collision testing but integrated with the body code
def test11_Lidar_Integrated():
    swarmbody = Body.SwarmBody();
    lastcol = True;
    #ldtmer = 0;
    ldlim = 0.7; # 2 second move
    mml = 15;
    swarmbody.duty_cycle = 0.3;
    while True:
        l1, l2, l3, l4 = swarmbody.l1,swarmbody.l2,swarmbody.l3,swarmbody.l4,;
        print(str(l1) + " " + str(l2) + " " + str(l3) + " " + str(l4))


        if l1 < mml or l2 < mml or l3 < mml or l4 < mml:
            ldtmer = 10;
            chrono2.reset()
            chrono2.start()

        if chrono2.read()<ldlim:
            if lastcol == False:
                stop_all();
                swarmbody.move_back();
                print("back")
            lastcol = True;

        else:
            if lastcol == True:
                stop_all();
                lastcol = False;
                swarmbody.move_forward();
                print("forward")
            lastcol = False;
            chrono2.stop()


#Running robot behaviour with sensors working
def test12_behav_sensors():
    #pycom.heartbeat(False)
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
    #Get initial Position
    position = swarmbody.get_pos();

    #Was 10, 80, Moves to the destination on a _thread, assume starting angle is 0
    #START THIS ON A NEW THREAD , all future calls must be thread calls
    #If PID_control reaches its destination or ends a collision we need to kill it
    PID_thread = _thread.start_new_thread(swarmbody.PID_movement,(swarmbeh.Target_Destination[0],swarmbeh.Target_Destination[1],(position[0], position[1]),position[2]))

    #Start the main Loop
    while True:

        #If we have arrived at our destination
        if swarmbody.Arrival_Flag == True:
            swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
            #ROTATE BACK TO ZERO ? ROTATE TO NEAREST 90 ? FEED THIS INTO NEXT PID ?
            #For safety kill the PID THREAD
            position = swarmbody.get_pos();
            PID_thread.kill();
            #PID_thread = _thread.start_new_thread(body.PID_control,(xdes=swarmbeh.Target_Destination[0], y_des=swarmbeh.Target_Destination[1], starting_coordinate=(position[0], position[1]), starting_angle=position[2]))

        elif swarmbody.Collision_Flag == True:
            swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
            #ROTATE BACK TO ZERO ? ROTATE TO NEAREST 90 ? FEED THIS INTO NEXT PID ?
            position = swarmbody.get_pos();
            PID_thread.kill();
            #PID_thread = _thread.start_new_thread(body.PID_control,(xdes=swarmbeh.Target_Destination[0], y_des=swarmbeh.Target_Destination[1], starting_coordinate=(position[0], position[1]), starting_angle=position[2]))
        #Run bluetooth and other code
        else:
            swarmbeh.Set_InternalXY(swarmbody.x,swarmbody.y);
            swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
            swarmbeh.Check_New_Grid_Cell_Handle(swarmbody,swarmbt);

#A test to see if the robot can move to four coordinates, this before integrating behaviour
def test_13_four_coords():
    body = Body.SwarmBody()
    body.duty_cycle = 0.5;


    complete = False
    print("[+] Setting Timer")

    while complete == False:
        time.sleep(1)

        if body._get_pos == 1 and body.gyro_data != 0:

            #input("Enter character to find coordinate: ")
            position = body.get_pos()
            print(position)
            body.PID_movement(50, 50, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
            position = body.get_pos()
            #break
            print("Coordinate: (",position[0],",",position[1],")")
            complete = True
            print("Reached the coordinate! wooooo")
            body.PID_movement(60, 50, starting_coordinate=(position[0],position[1]), starting_angle=position[2])

            position = body.get_pos()
            #break
            print("Coordinate: (",position[0],",",position[1],")")
            complete = True
            print("Reached the coordinate! wooooo")
            body.PID_movement(60, 60, starting_coordinate=(position[0],position[1]), starting_angle=position[2])

            position = body.get_pos()
            #break
            print("Coordinate: (",position[0],",",position[1],")")
            complete = True
            print("Reached the coordinate! wooooo")
            body.PID_movement(70, 70, starting_coordinate=(position[0],position[1]), starting_angle=position[2])

            position = body.get_pos()
            #break
            print("Coordinate: (",position[0],",",position[1],")")
            complete = True
            print("Reached the coordinate! wooooo")
            body.PID_movement(70, 80, starting_coordinate=(position[0],position[1]), starting_angle=position[2])

            position = body.get_pos()
            #break
            print("Coordinate: (",position[0],",",position[1],")")
            complete = True
            print("Reached the coordinate! wooooo")
            body.PID_movement(50, 50, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
            print("Coordinate: (",position[0],",",position[1],")")
            complete = True
            print("Reached the coordinate! wooooo")

        else:
            print("don't worry! I'm 22!!")


def test_16_no_thread_beh():
    body = Body.SwarmBody()
    body.duty_cycle = 0.5;
    body.battery = 100;
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    swarmbt.test_transmit();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,body);

    complete = False
    print("[+] Setting Timer")

    while complete == False:
        time.sleep(1)

        if body._get_pos == 1 and body.gyro_data != 0:


            swarmbeh.Choose_Target_Square(swarmbt,body);
            print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
            #input("Enter character to find coordinate: ")
            position = body.get_pos()
            print(position)
            body.PID_movement((swarmbeh.Target_Destination[0]+5)*10, (swarmbeh.Target_Destination[1]+5)*10, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
            position = body.get_pos()
            print("Reached the coordinate! wooooo")
            swarmbeh.Increment_Bounty_Tiles(1);
            swarmbeh.Set_InternalXY(swarmbeh.Target_Destination[0]*300,swarmbeh.Target_Destination[1]*300);
            swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(body,swarmbt);
            swarmbeh.Choose_Target_Square(swarmbt,body);
            #body.PID_movement((swarmbeh.Target_Destination[0]+5)*10, (swarmbeh.Target_Destination[1]+5)*10, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
            #swarmbeh.Increment_Bounty_Tiles(1);
            #swarmbeh.Set_InternalXY(swarmbeh.Target_Destination[0]*300,swarmbeh.Target_Destination[1]*300);
            #swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(body,swarmbt);
            #break
            print("Reached the coordinate! wooooo")
            #complete = True;


        else:
            print("don't worry! I'm 22!!")




def test_14_integration_testing():
    swarmbody = Body.SwarmBody();
    swarmbody.battery = 100;


    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    swarmbt.test_transmit();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
    Check = False;
    swarmbody.duty_cycle = 0.5;
    #swarmbody.V = 60;
    #swarmbody.KD = (0,1)
    while Check == False:
        if swarmbody._get_pos == 1 and swarmbody.gyro_data != 0:
            X = 0;
            Y = 0;
            #print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
            #Get initial Position
            position = swarmbody.get_pos();
            Check = True;
    #Was 10, 80, Moves to the destination on a _thread, assume starting angle is 0
    #START THIS ON A NEW THREAD , all future calls must be thread calls
    #If PID_control reaches its destination or ends a collision we need to kill it
    print("begin, coords, X:" + str((swarmbeh.Target_Destination[0]+5)*10) + " Y: " + str((swarmbeh.Target_Destination[1]+5)*10));
    #swarmbody.PID_movement((swarmbeh.Target_Destination[0]+4)*10, (swarmbeh.Target_Destination[1]+5)*10, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
    #swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
    #swarmbody.PID_movement((swarmbeh.Target_Destination[0]+4)*10, (swarmbeh.Target_Destination[1]+5)*10, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
    #swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
    #swarmbody.PID_movement((swarmbeh.Target_Destination[0]+4)*10, (swarmbeh.Target_Destination[1]+5)*10, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
    position = swarmbody.get_pos()
    swarmbody.PID_movement(50, 50, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
    position = swarmbody.get_pos()
    swarmbody.PID_movement(50, 60, starting_coordinate=(position[0],position[1]), starting_angle=position[2])


    #PID_thread = _thread.start_new_thread(swarmbody.PID_movement,((swarmbeh.Target_Destination[0]+4)*10,(swarmbeh.Target_Destination[1]+4)*10,(position[0], position[1]),position[2]))
    #PID_thread = _thread.start_new_thread(swarmbody.PID_movement,(50,50,(position[0], position[1]),position[2]))

def test_15_full_nt():
    swarmbody = Body.SwarmBody();
    swarmbody.battery = 100;


    #Initalise a bluetooth controller
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    swarmbt.test_transmit();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
    Check = False;
    while Check == False:
        if swarmbody._get_pos == 1 and swarmbody.gyro_data != 0:
            X = 0;
            Y = 0;
            #print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
            #Get initial Position
            position = swarmbody.get_pos();
            Check = True;
    #Was 10, 80, Moves to the destination on a _thread, assume starting angle is 0
    #START THIS ON A NEW THREAD , all future calls must be thread calls
    #If PID_control reaches its destination or ends a collision we need to kill it
    print("begin, coords, X:" + str((swarmbeh.Target_Destination[0]+5)*10) + " Y: " + str((swarmbeh.Target_Destination[1]+5)*10));
    PID_thread = _thread.start_new_thread(swarmbody.PID_movement,((swarmbeh.Target_Destination[0]+4)*10,(swarmbeh.Target_Destination[1]+4)*10,(position[0], position[1]),position[2]))
    #PID_thread = _thread.start_new_thread(swarmbody.PID_movement,(50,50,(position[0], position[1]),position[2]))

    while True:
        #If we have arrived at our destination
        if swarmbody.Arrival_Flag == True:
            swarmbody.Arrival_Flag = False;
            swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
            #ROTATE BACK TO ZERO ? ROTATE TO NEAREST 90 ? FEED THIS INTO NEXT PID ?
            #For safety kill the PID THREAD
            position = swarmbody.get_pos();
            PID_thread.kill();
            PID_thread = _thread.start_new_thread(swarmbody.PID_movement,((swarmbeh.Target_Destination[0]+4)*10,(swarmbeh.Target_Destination[1]+4)*10,(position[0], position[1]),position[2]))

        elif swarmbody.Collision_Flag == True:
            swarmbody.Collision_Flag = False;
            swarmbeh.Choose_Target_Square(swarmbt,swarmbody);
            #ROTATE BACK TO ZERO ? ROTATE TO NEAREST 90 ? FEED THIS INTO NEXT PID ?
            position = swarmbody.get_pos();
            PID_thread.kill();
            PID_thread = _thread.start_new_thread(swarmbody.PID_movement,((swarmbeh.Target_Destination[0]+4)*10,(swarmbeh.Target_Destination[1]+4)*10,(position[0], position[1]),position[2]))
        #Run bluetooth and other code
        else:
            swarmbeh.Set_InternalXY(swarmbody.x,swarmbody.y);
            swarmbt.Handle_Bluetooth_Behaviour(swarmbeh,False);
            swarmbeh.Check_New_Grid_Cell_Handle(swarmbody,swarmbt);

#gets solar panel readings
def test_17_solar_simple():
    while 1==1:
        swarmbody = Body.SwarmBody();

        vn = swarmbody.get_solar_panel_vol();
        print(vn);

def test_18_behvnsolar():
    body = Body.SwarmBody()
    body.duty_cycle = 0.5;
    body.battery = 100;
    swarmbt = Bluetooth_Comms.SwarmBluetooth();
    #Initialise a behaviour controller
    swarmbeh = Behaviour.SwarmBehaviour();
    swarmbt.test_transmit();
    #Choose an initial destination
    swarmbeh.Choose_Target_Square(swarmbt,body);

    complete = False
    print("[+] Setting Timer")

    while complete == False:
        time.sleep(1)

        if body._get_pos == 1 and body.gyro_data != 0:


            swarmbeh.Choose_Target_Square(swarmbt,body);
            print("X:" + str(swarmbeh.Target_Destination[0]) + "Y:" + str(swarmbeh.Target_Destination[1]));
            #input("Enter character to find coordinate: ")
            position = body.get_pos()
            print(position)
            body.PID_movement((swarmbeh.Target_Destination[0]+5)*10, (swarmbeh.Target_Destination[1]+5)*10, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
            position = body.get_pos()
            print("Reached the coordinate! wooooo")
            body.battery -= 21;
            swarmbeh.Increment_Bounty_Tiles(1);
            swarmbeh.Set_InternalXY(swarmbeh.Target_Destination[0]*300,swarmbeh.Target_Destination[1]*300);
            swarmbeh.Check_New_Grid_Cell_Handle(body,swarmbt);
            swarmbeh.Choose_Target_Square(swarmbt,body);
            #body.PID_movement((swarmbeh.Target_Destination[0]+5)*10, (swarmbeh.Target_Destination[1]+5)*10, starting_coordinate=(position[0],position[1]), starting_angle=position[2])
            #swarmbeh.Increment_Bounty_Tiles(1);
            #swarmbeh.Set_InternalXY(swarmbeh.Target_Destination[0]*300,swarmbeh.Target_Destination[1]*300);
            #swarmbeh.Check_New_Grid_Cell_Handle_NOSENSORS(body,swarmbt);
            #break
            print("Reached the coordinate! wooooo")
            #complete = True;


        else:
            print("don't worry! I'm 22!!")
'''
def test_18b_original_solar():
        body = SwarmBody()

    complete = False
    print("[+] Setting Timer")

    while complete == False:
        time.sleep(1)

        if body._get_pos == 1 and body.gyro_data != 0:
            complete = True


            ## Analogue Solar Voltage Reading ##
            adc = ADC(bits = 10)
            apin = adc.channel(pin = 'P15')     #reads pin 15 for battery output voltage
            ## Set ICS ##
            vMax = 0
            tMax = 0

            Vin = apin()

            # checks if the input from the solar panel is greater than / equal to the stored highest and stores the voltage and position.
            if(Vin >= vMax):
                vMax = Vin
                light_pos = body.get_pos()
                light_x = light_pos[0]
                light_y = light_pos[1]



         #check battery level - if lower than threshold go to coordinate
           apin = adc.channel(pin = 'PXX')     #reads pin XX for battery output voltage
           vBatt = apin()
           vThresh = 716                       #threshold voltage is about 3.5v here which is 716 binary with a 5v adc (is the output from adc binary?)

            #create moving average filter to smooth the battery voltage reading
            #va5 = va4
            #va4 = va3
            #va3 = va2
            #va2 = va1
            #va1 = vBatt
            #vFil = (va1+va2+va3+va4+va5)/5

            #using arrays           unsure if this works properly       will have to put a delay on the battery level with this due to rise up
                N = 5
                for j in range(N-1):
                    va[N-j] = va[j-1]       #start at va[5] & stop at va[1]
                va[0] = vBatt
                #finding moving average voltage
                vo = 0
                for h in range(N):      #calculates the moving average
                    vo = vo + (va[h]/N)


            #battery percentage
            #battPerc = (vo/4)*100   #using full range of 0-4v
            battPerc = ((vo-3.3)/0.7)*100    #using range between 3.3v and 4v as this is the operating region

            if(vo < vThresh):
                # move to latest light source coordinate
                current_pos = body.get_pos()
                current_angle = self.gyro_data
                PID_movement(light_x,light_y, starting_coordinate = (current_pos[0],current_pos[1]),starting_angle = current_angle)




        else:
            print("don't worry! we're still here (in main)")
'''
if __name__ == "__main__":
    ##Swarmbot is initialised
    #swarmbot = SwarmBot.SwarmBot()
    #swarmbot.alive()
    #swarmbt = Bluetooth_Comms.SwarmBluetooth();

    #test_14_integration_testing();
    #test_16_no_thread_beh();
    test_13_four_coords();

    pass
    #swarmbeh = Behaviour.SwarmBehaviour();
    #print("SwarmBot is Testing -_-");
    #test7_both_int();
