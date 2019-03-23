#Required Bluetooth comms fucntions

import pycom
import uos
import Body
import Network
from network import Bluetooth
import ubinascii

class SwarmBluetooth(Body.SwarmBody, Network.SwarmNetwork):
    def __init__(self):
        #Body.SwarmBody.__init__(self)
        #Network.SwarmNetwork.__init__(self)
        self.bluetooth = Bluetooth();
        self.Collision_Timer = 0;
        self.Tile_Transmit_Timer = 2;
        self.bl_threshold = -35;
        self.RepulsionTime = 2000;
        pass


    #Transmits Information about a Tile that a robot is traversing
    #We don't actually need to send temperature information in this message as other bots need not know
    def Start_Transmit_Tile_Update(self,RX,RY,Luminosity,Time_Steps):
        #This statment may be not good, current unsure
        #self.bluetooth.stopscan();
        print("Tile Transmit Update" + str(RX)+"/"+str(RY));

        #Sets the advertisment that we want, the method work up to 99, uses 01, 10

        if RX < 10:
            RXM = "0"+str(RX);
        else:
            RXM = str(RX);

        if RY < 10:
            RYM = "0"+str(RY);
        else:
            RYM = str(RY);


        ss = RXM + RYM +str(Luminosity);
        #print(ss);
        self.bluetooth.set_advertisement(name="a_mp", manufacturer_data="l", service_data=ss)
        self.bluetooth.advertise(True)
        #Sets teh timer for how long we should transmit
        self.Tile_Transmit_Timer = 200;
        return -1;


    def test_transmit(self):
        self.bluetooth.set_advertisement(name="a", manufacturer_data="l", service_data="99999")
        self.bluetooth.advertise(True)
    #Broadcasts the selection of a tile as a robots destination
    def Broadcast_Tile_Selection(self,Target_Destination,State):

        RX = Target_Destination[0];
        RY = Target_Destination[1];
        print("Broadcasting Tile Selection" + str(RX)+"/"+str(RY));

        #Sets the advertisment that we want, the method work up to 99, uses 01, 10

        if RX < 10:
            RXM = "0"+str(RX);
        else:
            RXM = str(RX);

        if RY < 10:
            RYM = "0"+str(RY);
        else:
            RYM = str(RY);


        mes = RXM + RYM +str(State);

        self.bluetooth.set_advertisement(name="a_tg", manufacturer_data="l", service_data=mes)
        self.bluetooth.advertise(True)
        self.Tile_Transmit_Timer = 50;
        return -1;


    #Handles Transmission and Listening Decisions For a given cycle
    #Needs to be run each cycle
    def Handle_Bluetooth_Behaviour(self,Swarmbehv_obj,print_boolean,SwarmBody_2):
        if self.Collision_Timer > 0:
            self.Collision_Timer-=1;

        #Transmit if transmission timer is active
        #print(str(self.Tile_Transmit_Timer));
        if self.Tile_Transmit_Timer > 1:
            self.Tile_Transmit_Timer-=1;
            #self.bluetooth.stopscan();
            #print("Decrementing")
        #If stopping transmission
        elif self.Tile_Transmit_Timer == 1:
            self.bluetooth.advertise(False)
            self.Tile_Transmit_Timer-=1;
            #Start Scanning
            if self.bluetooth.isscanning() == False:
                self.bluetooth.start_scan(-1);
            #print("Starting A Scan")
        #If not transmitting
        else:

            #If we are scanning for bluetooth transmissions
            if self.bluetooth.isscanning():
                #print("Scanning");
                #Continue to do it
                adv = self.bluetooth.get_adv()
                if adv:
                    # try to get the complete name

                    if print_boolean == True:
                        pass
                        #print(self.bluetooth.resolve_adv_data(adv.data, Bluetooth.ADV_NAME_CMPL))
                    name = self.bluetooth.resolve_adv_data(adv.data, Bluetooth.ADV_NAME_CMPL);
                    mfg_data = self.bluetooth.resolve_adv_data(adv.data, Bluetooth.ADV_MANUFACTURER_DATA)
                    adv_mes = self.bluetooth.resolve_adv_data(adv.data, Bluetooth.ADV_SERVICE_DATA)
                    if print_boolean == True:
                        pass
                        #print(adv_mes);
                    if adv_mes:
                        if print_boolean == True:
                            print("MES!")

                    if mfg_data:
                        if print_boolean == True:
                            pass
                            #print(ubinascii.hexlify(mfg_data))

                    if adv_mes and name:
                        bl_strength = adv[3];
                        #If the strength is past the bl_threshold
                        SwarmBody_2.Last_bt_str = bl_strength;
                        if bl_strength > self.bl_threshold:
                            #We start our collision reverse movement
                            if self.Collision_Timer == 0:
                                self.Collision_Timer = self.RepulsionTime;
                            #Ths code wll be req n future
                            #Unless we are alreay in one, then we cancel it and restart normal movement
                            #else:
                                #self.Collision_Timer = 0;


                        if print_boolean == True:
                            pass
                            #print(ubinascii.hexlify(adv_mes))
                            #print(adv_mes)

                        #If meesage is an intent update
                        if name == "a_tg":
                            #if print_boolean == True:
                            #print("target recieved!" + str(bl_strength))

                            #do it
                            hexd = ubinascii.hexlify(adv_mes);
                            mx = (int(adv_mes[0])-48)*10+(int(adv_mes[1])-48);
                            my = (int(adv_mes[2])-48)*10+(int(adv_mes[3])-48);
                            state = int(adv_mes[4])-48;
                            if print_boolean == True:
                                print(mx);
                                print(my);
                                print(state);
                            #If all of them are integers
                            if isinstance(mx,int) and isinstance(my,int) and isinstance(state,int):
                                Swarmbehv_obj.Map_Assignement[mx][my] = state;
                                if print_boolean == True:
                                    #Swarmbehv_obj.Display_Map(Swarmbehv_obj.Map_Light);
                                    pass


                        elif name == "a_mp":
                            if print_boolean == True:
                                print("mapping recieved !")
                            #If message is a tile update
                            #Get coords of square
                            cx = (int(adv_mes[0])-48)*10+(int(adv_mes[1])-48);
                            cy = (int(adv_mes[2])-48)*10+(int(adv_mes[3])-48);
                            #create temp string
                            lumin_s = 0;
                            for i in range(4,len(adv_mes)):
                                lumin_s += (10**(len(adv_mes)-i-1))*(int(adv_mes[i])-48);
                            #make temp float
                            lumin_s = float(lumin_s);
                            if print_boolean == True:
                                print(cx)
                                print(cy)
                                print(lumin_s)
                            #If cx and cy are integers
                            if isinstance(cx,int) and isinstance(cy,int) and cx > 0 and cy > 0:
                                Swarmbehv_obj.Map_Light[cx][cy] = lumin_s;
                                Swarmbehv_obj.Map_Bounty[cx][cy] = 0;
                                #Swarmbehv_obj.Area_Matrix[cx][cy] = 1;
                                #print(Swarmbehv_obj.Area_Matrix);
                                if print_boolean == True:
                                    Swarmbehv_obj.Display_Map(Swarmbehv_obj.Map_Light);


    #Used to run handle bluetooth behaviour on a thread
    def Handle_Bluetooth_Behaviour_Continuous(self,Swarmbehv_obj,print_boolean):
        while True:
            self.Handle_Bluetooth_Behaviour(Swarmbehv_obj,print_boolean);
