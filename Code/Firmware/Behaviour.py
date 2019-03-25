import Body
import Network
import pycom
import uos
import Bluetooth_Comms
import math
import wifi


class SwarmBehaviour(Body.SwarmBody, Network.SwarmNetwork, Bluetooth_Comms.SwarmBluetooth):
    def __init__(self):
        #Body.SwarmBody.__init__(self)
        self.network = Network.SwarmNetwork()
        self.network.send_debug_info("[+] Succesfully setup network")
        #Bluetooth_Comms.SwarmBluetooth.__init__(self)

        self.Collision_Timer = 0;
        self.Target_Destination = [0,0];
        self.Light_Weighting = 10;

        #Arena Definition, in mm
        self.Arena_X_Mm = 3000;
        self.Arena_Y_Mm = 3000;

        self.Arena_Grid_Size_X = 600;
        self.Arena_Grid_Size_Y = 600;

        self.Tile_Num_X = round(self.Arena_X_Mm/self.Arena_Grid_Size_X);
        self.Tile_Num_Y = round(self.Arena_Y_Mm/self.Arena_Grid_Size_Y);

        self.Internal_X = 0;
        self.Internal_Y = 0;

        self.Last_Grid_Cell_X = -1;
        self.Last_Grid_Cell_Y = -1;

        self.Current_Grid_Cell_X = 0;
        self.Current_Grid_Cell_Y = 0;

        self.speed = 1;
        self.delta_dist = 1;

        self.Map_Temp = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];
        self.Map_Bounty = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];
        self.Map_Assignement = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];
        self.Map_Light = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];

        self.Temp_Readings = [[0]*3 for _ in range(26)];
        self.Temp_Counter = 0;
        self.Temp_Readings_dict = {"x": [],
                                    "y": [],
                                    "temp": []}

        pass
    #Outputs a map on the terminal
    def Display_Map(self,Map):
        tms = "";
        for i in range (0,self.Tile_Num_X):#
            if i != 0:
                tms +="\n";
            for j in range(0,self.Tile_Num_Y):
                tms += str(Map[i][j])+",";
        print(tms);

    def Display_Temps(self,Map):
        tms = "";
        for i in range (0,20):#
            if i != 0:
                tms +="\n";
            for j in range(0,3):
                tms += str(Map[i][j])+",";
        print(tms);


    def get_state(self):
        state = self._get_state_info()

        return state

    #Current XY can be passed to the behaviour so decisions can be made
    def Set_InternalXY(self,X,Y):
        self.Internal_X = X;
        self.Internal_Y = Y;

    #Using the bounty and light maps chooses a target square
    #This will update internal variables of passed Swarmbot_obj and also return the coords.
    def Choose_Target_Square(self,Bluetooth_obj,Swarmbot_obj):
        tempbounty = 0;
        self.Map_Assignement[self.Target_Destination[0]][self.Target_Destination[1]] = 0;
        Bluetooth_obj.Broadcast_Tile_Selection(self.Target_Destination,0);
        print("Battery",Swarmbot_obj.battery)
        for i in range(0,self.Tile_Num_X):
            for j in range(0,self.Tile_Num_Y):
                bt = self.Map_Bounty[i][j];
                bt_p_light = bt# + self.Map_Light[i][j] * (1/Swarmbot_obj.battery+1) *self.Light_Weighting;
                if Swarmbot_obj.battery < 10:
                    bt_p_light = self.Map_Light[i][j];# * (1/Swarmbot_obj.battery+1) *self.Light_Weighting;
                    #Finding distance between us and a target tiles
                xl = abs(i*self.Arena_Grid_Size_X - self.Internal_X);
                yl = abs(j*self.Arena_Grid_Size_Y - self.Internal_Y);
                #Sqrt
                dist1 = (((xl*xl + yl*yl)**(1/2.0))+1)**0.5;
                if Swarmbot_obj.battery < 10:
                    dist1 = 1;
                bt_dmod = (bt_p_light/dist1)
                    #Unassign ourselves from our last target
                #If the bounty is higher, its already not assigned or its already our destination
                if bt_dmod > tempbounty and (self.Map_Assignement[i][j] != 1 or (self.Target_Destination[0] == i and self.Target_Destination[1] == j)):
                    tempbounty = bt_dmod
                    self.Target_Destination[0] = i;
                    self.Target_Destination[1] = j;
                    #self.Map_Assignement[i][j] = 1;

        ##Alerts other robos' of its tile intent.
        #We can potentailly just make this external as internally there are some problems
        self.Map_Assignement[self.Target_Destination[0]][self.Target_Destination[1]] = 1;
        Bluetooth_obj.Broadcast_Tile_Selection(self.Target_Destination,1);


        return [self.Target_Destination[0],self.Target_Destination[1]];

    #A simpler variant of the choose target square system to avoid gyro problems
    def Choose_Target_Square_Simple(self,Bluetooth_obj,Swarmbot_obj):
        tempbounty = 0;
        self.Map_Assignement[self.Target_Destination[0]][self.Target_Destination[1]] = 0;
        Bluetooth_obj.Broadcast_Tile_Selection(self.Target_Destination,0);
        for K in range(0,4):
            #N
            if K==0:
                i = self.Target_Destination[0];
                j = self.Target_Destination[1]+1;
            #E
            elif K==1:
                i = self.Target_Destination[0]+1;
                j = self.Target_Destination[1];
            #S
            elif K==2:
                i = self.Target_Destination[0];
                j = self.Target_Destination[1]-1;
            #W
            elif K==3:
                i = self.Target_Destination[0]-1;
                j = self.Target_Destination[1];

            #if it is a tile within the grid then proceed
            if i > 9 or i < 0 or j > 9 or j < 0:
                bt = self.Map_Bounty[i][j];
                bt_p_light = bt# + self.Map_Light[i][j] * (1/Swarmbot_obj.battery+1) *self.Light_Weighting;
                if Swarmbot_obj.battery < 10:
                    bt_p_light = self.Map_Light[i][j] * (1/Swarmbot_obj.battery+1) *self.Light_Weighting;
                    #Finding distance between us and a target tiles
                xl = abs(i*self.Arena_Grid_Size_X - self.Internal_X);
                yl = abs(j*self.Arena_Grid_Size_Y - self.Internal_Y);
                #Sqrt
                dist1 = (((xl*xl + yl*yl)**(1/2.0))+1)*0.5;
                bt_dmod = (bt_p_light/dist1)
                    #Unassign ourselves from our last target
                #If the bounty is higher, its already not assigned or its already our destination
                if bt_dmod > tempbounty and (self.Map_Assignement[i][j] != 1 or (self.Target_Destination[0] == i and self.Target_Destination[1] == j)):
                    tempbounty = bt_dmod
                    self.Target_Destination[0] = i;
                    self.Target_Destination[1] = j;
                    #self.Map_Assignement[i][j] = 1;

        ##Alerts other robos' of its tile intent.
        #We can potentailly just make this external as internally there are some problems
        self.Map_Assignement[self.Target_Destination[0]][self.Target_Destination[1]] = 1;
        Bluetooth_obj.Broadcast_Tile_Selection(self.Target_Destination,1);


        return [self.Target_Destination[0],self.Target_Destination[1]];

    #Increments the bounty tiles by 1 and returns the incremented map;
    def Increment_Bounty_Tiles(self,increm):
        for i in range(0,self.Tile_Num_X):
       			for j in range(0,self.Tile_Num_Y):
       				self.Map_Bounty[i][j] += increm;
        return self.Map_Bounty;

    def Ring_Null(self):
        for i in range(0,self.Tile_Num_X):
            for j in range(0,self.Tile_Num_Y):
                if i == 0 or j == 0 or i == 9 or j == 9:
                    self.Map_Bounty[i][j] = 0;


    #Checks if the robot is in a new grid cell and if so executes required code


    def Check_New_Grid_Cell_Handle(self,Swarmbot_obj,Bluetooth_obj):
        1==1;
        #If we are in a new Grid Cell
        self.Current_Grid_Cell_X = math.floor(self.Internal_X/self.Arena_Grid_Size_X);
        self.Current_Grid_Cell_Y = math.floor(self.Internal_Y/self.Arena_Grid_Size_Y);
        if self.Current_Grid_Cell_X != self.Last_Grid_Cell_X or self.Current_Grid_Cell_Y != self.Last_Grid_Cell_Y:
            print("New Cell!"+str(self.Current_Grid_Cell_X)+"/"+str(self.Current_Grid_Cell_Y))
            #We get the temp of the Cell & Light

            Current_Grid_Cell_Temp = swarmbody.get_temp();
            Current_Grid_Cell_Luminosity = swarmbody.S_apin.voltage();
            #Find Our Definite coords
            Real_X = self.Internal_X;
            Real_Y = self.Internal_Y;

            #convert coords into a grid square
            Real_Grid_Square_X = math.floor(Real_X/self.Arena_Grid_Size_X);
            Real_Grid_Square_Y = math.floor(Real_Y/self.Arena_Grid_Size_Y);
            #Probably a good place to correct our internal coords
            self.Internal_X = Real_X;
            self.Internal_Y = Real_Y;

            #Update your internal grid map
            self.Map_Bounty[Real_Grid_Square_X][Real_Grid_Square_Y] = 0;
            self.Map_Temp[Real_Grid_Square_X][Real_Grid_Square_Y] = Current_Grid_Cell_Temp;
            self.Map_Light[Real_Grid_Square_X][Real_Grid_Square_Y] = Current_Grid_Cell_Luminosity;
            #Transmit the new information, starts transmission

            Bluetooth_obj.Start_Transmit_Tile_Update(Real_Grid_Square_X,Real_Grid_Square_Y,Current_Grid_Cell_Luminosity,15);


            #If the grid cell we are in is the target then we choose a new Target_Destination
            if self.Current_Grid_Cell_X == self.Target_Destination[0] and self.Current_Grid_Cell_Y == self.Target_Destination[1]:
                self.Target_Destination = self.Choose_Target_Square(Bluetooth_obj,Swarmbot_obj);
                print("Chosen new t dest!" + str(self.Target_Destination[0])+"/"+str(self.Target_Destination[1]));
                self.Display_Map(self.Map_Light);
                #Wipe grid to prevent poor comms trash buildup
                self.Map_Assignement = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];

        self.Last_Grid_Cell_X = self.Current_Grid_Cell_X;
        self.Last_Grid_Cell_Y = self.Current_Grid_Cell_Y;


    def Check_New_Grid_Cell_Handle_NOSENSORS(self,Swarmbot_obj,Bluetooth_obj):
        1==1;
        #If we are in a new Grid Cell
        self.Current_Grid_Cell_X = math.floor(self.Internal_X/self.Arena_Grid_Size_X);
        self.Current_Grid_Cell_Y = math.floor(self.Internal_Y/self.Arena_Grid_Size_Y);
        if self.Current_Grid_Cell_X != self.Last_Grid_Cell_X or self.Current_Grid_Cell_Y != self.Last_Grid_Cell_Y:
            print("New Cell!"+str(self.Current_Grid_Cell_X)+"/"+str(self.Current_Grid_Cell_Y))
            #We get the temp of the Cell & Light

            Current_Grid_Cell_Temp = Swarmbot_obj.get_temp();
            Current_Grid_Cell_Luminosity = Swarmbot_obj.S_apin.voltage();

            pos = Swarmbot_obj.get_pos();
            self.Temp_Readings[self.Temp_Counter][0] = pos[0]
            self.Temp_Readings[self.Temp_Counter][1] = pos[1]
            self.Temp_Readings[self.Temp_Counter][2] = Current_Grid_Cell_Temp;
            self.network.send_debug_info("[+] About to send x: {} y: {} temp: {} bat: {}".format(pos[0], pos[1], Current_Grid_Cell_Temp, 3.3))
            self.network._send_state_wifi(pos[0], pos[1], Current_Grid_Cell_Temp, 3.3)
            #self.Temp_Readings_dict["x"].append(pos[0])
            #self.Temp_Readings_dict["y"].append(pos[1])
            #self.Temp_Readings_dict["temp"].append(Current_Grid_Cell_Temp)
            #self.Temp_Counter += 1;
            #if self.Temp_Counter == 25:
            #    self.Temp_Counter=0
            #    for i in range (0, len(self.Temp_Readings_dict["x"])):
            #        print(self.Temp_Readings_dict["x"][i])
            #        print(self.Temp_Readings_dict["y"][i])
            #        print(self.Temp_Readings_dict["temp"][i])
            #        self.network._send_state_wifi(self.Temp_Readings_dict["x"][i],self.Temp_Readings_dict["y"][i] , self.Temp_Readings_dict["temp"][i], 3.3)
            self.Display_Temps(self.Temp_Readings);

            print("Light",Current_Grid_Cell_Luminosity);
            print("Temp",Current_Grid_Cell_Temp);
            #Find Our Definite coords
            Real_X = self.Internal_X;
            Real_Y = self.Internal_Y;

            #convert coords into a grid square
            Real_Grid_Square_X = math.floor(Real_X/self.Arena_Grid_Size_X);
            Real_Grid_Square_Y = math.floor(Real_Y/self.Arena_Grid_Size_Y);
            #Probably a good place to correct our internal coords
            self.Internal_X = Real_X;
            self.Internal_Y = Real_Y;

            #Update your internal grid map
            self.Map_Bounty[Real_Grid_Square_X][Real_Grid_Square_Y] = 0;
            self.Map_Temp[Real_Grid_Square_X][Real_Grid_Square_Y] = Current_Grid_Cell_Temp;
            self.Map_Light[Real_Grid_Square_X][Real_Grid_Square_Y] = Current_Grid_Cell_Luminosity;
            #Transmit the new information, starts transmission

            Bluetooth_obj.Start_Transmit_Tile_Update(Real_Grid_Square_X,Real_Grid_Square_Y,Current_Grid_Cell_Luminosity,15);


            #If the grid cell we are in is the target then we choose a new Target_Destination
            if self.Current_Grid_Cell_X == self.Target_Destination[0] and self.Current_Grid_Cell_Y == self.Target_Destination[1]:
                self.Target_Destination = self.Choose_Target_Square(Bluetooth_obj,Swarmbot_obj);
                print("Chosen new t dest!" + str(self.Target_Destination[0])+"/"+str(self.Target_Destination[1]));
                self.Display_Map(self.Map_Light);
                #Wipe grid to prevent poor comms trash buildup
                self.Map_Assignement = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];

        self.Last_Grid_Cell_X = self.Current_Grid_Cell_X;
        self.Last_Grid_Cell_Y = self.Current_Grid_Cell_Y;
    #Does the checks required to perform the correct movement
    #A check that we are facng the right angle is required here
    def Robot_Movement_Behaviour(self,Swarmbot_obj):
        if self.Collision_Timer == 0:
            Swarmbot_obj.move_foward(self.speed,self.delta_dist);
        elif self.Collision_Timer > 1:
            #We decrement the collision timer
            self.Collision_Timer -= 1;
            #The robot moves away from its current target
            Swarmbot_obj.move_backward(self.speed,self.delta_dist);
        else:
            #If the collision timer is 1
            #Choose a new Target_Destination
            self.Choose_Target_Square(self.Map_Bounty);

'''
Temp_Readings = [[0]*3 for _ in range(20)];
print("Test")
print(Temp_Readings)
beh = SwarmBehaviour();
beh.Display_Temps(Temp_Readings)
'''
