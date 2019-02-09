import Body
import Network
import pycom
import uos
import Bluetooth_Comms



class SwarmBehaviour(Body.SwarmBody, Network.SwarmNetwork):
    def __init__(self):
        #Body.SwarmBody.__init__(self)
        #Network.SwarmNetwork.__init__(self)
        #Bluetooth_Comms.SwarmBluetooth.__init__(self)

        self.Collision_Timer = 0;
        self.Target_Destination = [0,0];
        self.Light_Weighting = 10;

        #Arena Definition, in mm
        self.Arena_X_Mm = 3000;
        self.Arena_Y_Mm = 3000;

        self.Arena_Grid_Size_X = 300;
        self.Arena_Grid_Size_Y = 300;

        self.Tile_Num_X = round(self.Arena_X_Mm/self.Arena_Grid_Size_X);
        self.Tile_Num_Y = round(self.Arena_Y_Mm/self.Arena_Grid_Size_Y);

        self.Internal_X = 0;
        self.Internal_Y = 0;

        self.Current_Grid_Cell_X = 0;
        self.Current_Grid_Cell_Y = 0;

        self.speed = 1;
        self.delta_dist = 1;

        self.Map_Temp = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];
        self.Map_Bounty = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];
        self.Map_Assignement = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];
        self.Map_Light = [[0]*self.Tile_Num_X for _ in range(self.Tile_Num_Y)];

        pass

    #Outputs a map on the terminal
    def Display_Map(self,Map):
        tms = "";
        for i in range (0,Tile_Num_X):#
            if i != 0:
                tms +="\n";
            for j in range(0,Tile_Num_Y):
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
        self.Map_Assignement[self.Target_Destination[0],self.Target_Destination[1]] = 0;
        Bluetooth_obj.Broadcast_Tile_Selection(self.Target_Destination,0);
        for i in range(0,self.Tile_Num_X):
            for j in range(0,self.Tile_Num_Y):
                bt = self.Map_Bounty[i][j];
                bt_p_light = bt + self.Map_Light[i][j] * (1/Swarmbot_obj.battery) *self.Light_Weighting;
                if Swarmbot_obj.battery < 10:
                    bt_p_light = self.Map_Light[i][j] * (1/Swarmbot_obj.battery) *self.Light_Weighting;
                    #Finding distance between us and a target tiles
                    xl = i*self.Arena_Grid_Size_X - self.Internal_X;
                    yl = j*self.Arena_Grid_Size_Y - self.Internal_Y;
                    #Sqrt
                    dist1 = (xl*xl + yl*yl)**(1/2.0);
                    bt_dmod = (bt_p_light/dist1)
                    #Unassign ourselves from our last target

                if bt_dmod > tempBounty and (self.Map_Assignement[i][j] != 1 or (self.Target_Destination[0] == i and self.Target_Destination[1] == j)):
                    tempBounty = bt_dmod
                    self.Target_Destination[0] = i;
                    self.Target_Destination[1] = j;
                    self.Map_Assignement[i][j] = 1;

        ##Alerts other robos' of its tile intent.
        Bluetooth_obj.Broadcast_Tile_Selection(self.Target_Destination,1);


        return [self.Target_Destination[0],self.Target_Destination[1]];

        #Increments the bounty tiles by 1 and returns the incremented map;
        def Increment_Bounty_Tiles(self):
            for i in range(0,self.Tile_Num_X):
           			for j in range(0,self.Tile_Num_Y):
           				self.Map_Bounty[i][j] += 1;
            return self.Map_Bounty;

        #Checks if the robot is in a new grid cell and if so executes required code


        def Check_New_Grid_Cell_Handle(self,Swarmbot_obj,Bluetooth_obj):
            1==1;
            #If we are in a new Grid Cell
            self.Current_Grid_Cell_X = floor(self.Internal_X/self.Arena_Grid_Size_X);
            self.Current_Grid_Cell_Y = floor(self.Internal_Y/self.Arena_Grid_Size_Y);
            if self.Current_Grid_Cell_X != self.Last_Grid_Cell_X and self.Current_Grid_Cell_Y != self.Last_Grid_Cell_Y:
                #We get the temp of the Cell & Light

                Current_Grid_Cell_Temp = Swarmbot_obj.get_temp();
                Current_Grid_Cell_Luminosity = Swarmbot_obj.get_solar_panel_vol();
                #Find Our Definite coords
                Real_X,Real_Y = Swarmbot_obj.get_pos();

                #convert coords into a grid square
                Real_Grid_Square_X = floor(Real_X/Arena_Grid_Size_X);
                Real_Grid_Square_Y = floor(Real_Y/Arena_Grid_Size_Y);
                #Probably a good place to correct our internal coords
                self.Internal_X = Real_X;
                self.Internal_Y = Real_Y;

                #Update your internal grid map
                self.Map_Bounty[Real_Grid_Square_X][Real_Grid_Square_Y] = 0;
                self.Map_Temp[Real_Grid_Square_X][Real_Grid_Square_Y] = Current_Grid_Cell_Temp;
                self.Map_Light[Real_Grid_Square_X][Real_Grid_Square_Y] = Current_Grid_Cell_Luminosity;
                #Transmit the new information, starts transmission

                Bluetooth_obj.Start_Transmit_Tile_Update(Real_X,Real_Y,Current_Grid_Luminosity);


                #If the grid cell we are in is the target then we choose a new Target_Destination
                if self.Current_Grid_Cell_X == self.Target_Destination[0] and self.Current_Grid_Cell_Y == self.Target_Destination[1]:
                    self.Target_Destination = Choose_Target_Square(self.Map_Bounty);


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
