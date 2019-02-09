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
        pass


    #Transmits Information about a Tile that a robot is traversing
    #We don't actually need to send temperature information in this message as other bots need not know
    def Start_Transmit_Tile_Update(self,RX,RY,Luminosity,Time_Steps):
        print("Tile Transmit Update" + str(RX)+"/"+str(RY));
        #Enalbes Bluetooth
        self.bluetooth.advertise(True)
        #Sets the advertisment that we want, the method work up to 999, uses 001, 010, 100

        if RX < 10:
            RXM = "00"+str(RX);
        elif RX < 100:
            RXM = "0"+str(RX);
        else:
            RXM = str(RX);

        if RY < 10:
            RYM = "00"+str(RY);
        elif RY < 100:
            RYM = "0"+str(RY);
        else:
            RYM = str(RY);


        ss = RXM + "/" + RYM +"/"+str(Luminosity);

        self.bluetooth.set_advertisement(name="ad_map", manufacturer_data="lopy_v1", service_data="g")
        #Sets teh timer for how long we should transmit
        self.Tile_Transmit_Timer = Time_Steps;


    #Broadcasts the selection of a tile as a robots destination
    def Broadcast_Tile_Selection(self,Target_Destination,State):

        RX = Target_Destination[0];
        RY = Target_Destination[1];
        print("Broadcasting Tile Selection" + str(RX)+"/"+str(RY));
        #Enalbes Bluetooth
        self.bluetooth.advertise(True)
        #Sets the advertisment that we want, the method work up to 999, uses 001, 010, 100

        if RX < 10:
            RXM = "00"+str(RX);
        elif RX < 100:
            RXM = "0"+str(RX);
        else:
            RXM = str(RX);

        if RY < 10:
            RYM = "00"+str(RY);
        elif RY < 100:
            RYM = "0"+str(RY);
        else:
            RYM = str(RY);


        mes = RXM + "/" + RYM +"/"+str(State);

        self.bluetooth.set_advertisement(name="ad_targ", manufacturer_data="lopy_v1", service_data="b")
        self.bluetooth.advertise(True)
        self.Tile_Transmit_Timer = 15;
        return -1;


    #Handles Transmission and Listening Decisions For a given cycle
    #Needs to be run each cycle
    def Handle_Bluetooth_Behaviour(self,Swarmbehv_obj):
        #Transmit if transmission timer is active
        #print(str(self.Tile_Transmit_Timer));
        if self.Tile_Transmit_Timer > 1:
            self.Tile_Transmit_Timer-=1;
            #print("Decrementing")
        #If stopping transmission
        elif self.Tile_Transmit_Timer == 1:
            self.bluetooth.advertise(False)
            self.Tile_Transmit_Timer-=1;
            #Start Scanning
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
                    bl_strength = adv[3];
                    #If the strength is past the bl_threshold
                    if bl_strength > self.bl_threshold:
                        #We start our collision reverse movement
                        if self.Collision_Timer == 0:
                            self.Collision_Timer = 20;
                        #Unless we are alreay in one, then we cancel it and restart normal movement
                        else:
                            self.Collision_Timer = 0;
                    print(self.bluetooth.resolve_adv_data(adv.data, Bluetooth.ADV_NAME_CMPL))
                    name = self.bluetooth.resolve_adv_data(adv.data, Bluetooth.ADV_NAME_CMPL);
                    mfg_data = self.bluetooth.resolve_adv_data(adv.data, Bluetooth.ADV_MANUFACTURER_DATA)
                    adv_mes = self.bluetooth.resolve_adv_data(adv.data, Bluetooth.ADV_SERVICE_DATA)
                    

                    if mfg_data:
                        print(ubinascii.hexlify(mfg_data))

                    if adv_mes and name:
                        print(ubinascii.hexlify(adv_mes))
                        print(adv_mes)

                        #If meesage is an intent update
                        if name == "ad_targ":

                            #do it
                            mx = int(adv_mes[0]+adv_mes[1]+adv_mes[2])-48;
                            my = int(adv_mes[4]+adv_mes[5]+adv_mes[6])-48;
                            state = int(adv_mes[7])-48;
                            #If all of them are integers
                            if isinstance(mx,int) and isinstance(my,int) and isinstance(state,int):
                                Swarmbehv_obj.Map_Assignement[mx][my] = state;


                        elif name == "ad_map":
                            #If message is a tile update
                            #Get coords of square
                            cx = int(adv_mes[0]+adv_mes[1]+adv_mes[2])-48;
                            cy = int(adv_mes[4]+adv_mes[5]+adv_mes[6])-48;
                            #create temp string
                            for i in range(8,len(adv_mes)):
                                lumin_s += adv_mes[i];
                            #make temp float
                            lumin_s = float(lumin_s);
                            print(cx)
                            print(cy)
                            print(temp)
                            #If cx and cy are integers
                            if isinstance(cx,int) and isinstance(cy,int):
                                Swarmbehv_obj.Map_Light[cx][cy] = lumin_s;
                                Swarmbehv_obj.Map_Bounty[cx][cy] = 0;
                                Swarmbehv_obj.Area_Matrix[cx][cy] = 1;
                                print(Swarmbehv_obj.Area_Matrix);
