import pycom
import time
import utime
import machine
from machine import Pin
from machine import I2C
from machine import Timer
from micropython import const
from machine import PWM
import mpu6050
import sys
import VL53L0X
#from tmp102 import _tmp102

# Setting Constants for Serial Busses
I2C_BUS_0 = const(0)
I2C_BUS_1 = const(1)
LIDAR_DEFAULT_ADDR = const(0x29)
LIDAR_ADDR_REGISTER = const(0x8A)
LIDAR1_ADDR = const(0x15)
LIDAR2_ADDR = const(0x16)
LIDAR3_ADDR = const(0x17)
LIDAR4_ADDR = const(0x18)
TEMP_ADDR = const(0x48)


# Class for movement and Sensing of Robot
class SwarmBody():
    # Init function initialising pins and sensors
    def __init__(self, temp_SDA="P9", temp_SCL="P21", motor_pin1='P10', motor_pin2='P11', motor_pin3='P12', motor_pin4='P6',
                 lidar_SDA='P7',
                 lidar_SCL='P22',
                 lidar_DIO1='P2', lidar_DIO2='P3', lidar_DIO3='P4', lidar_DIO4='P5', gyro_SDA='P23', gyro_SCL='P22',
                 temp_AD='P13'):
        # self.initialise_motor(motor_pin1, motor_pin2, motor_pin3, motor_pin4)
        self.initialise_gyro(gyro_SDA, gyro_SCL)
        #self.initialise_lidar(lidar_SDA, lidar_SCL, lidar_DIO1, lidar_DIO2, lidar_DIO3, lidar_DIO4)
        #self.initialise_temp(temp_SDA, temp_SCL)

    def initialise_solar_panel_monitor(self, solar_panel_AD):
        self.solar_panel_AD = solar_panel_AD
        self.solar_panel_adc = machine.ADC()
        self.solar_panel_APIN = self.solar_panel_adc.channel(pin=self.solar_panel_AD)

    # TODO: Battery Monitor code
    # Function for initialising all pin functionality for lidar
    def _set_lidar(self, ID, lidar_DIO):
        if ID==1:
            self.lidar_DIO1.value(0)
            self.lidar_DIO1 = Pin(lidar_DIO, mode=Pin.IN)
            self.write_address(LIDAR1_ADDR)
            self.tof1 = VL53L0X.VL53L0X(i2c=self.lidar_I2C, address=LIDAR1_ADDR)
            device = self.lidar_I2C.scan()
            print(device)
        elif ID==2:
            self.lidar_DIO2.value(0)
            self.lidar_DIO2 = Pin(lidar_DIO, mode=Pin.IN)
            self.write_address(LIDAR2_ADDR)
            self.tof2 = VL53L0X.VL53L0X(i2c=self.lidar_I2C, address=LIDAR2_ADDR)
            device = self.lidar_I2C.scan()
            print(device)
        elif ID==3:
            self.lidar_DIO3.value(0)
            self.lidar_DIO3 = Pin(lidar_DIO, mode=Pin.IN)
            self.write_address(LIDAR3_ADDR)
            self.tof3 = VL53L0X.VL53L0X(i2c=self.lidar_I2C, address=LIDAR3_ADDR)
            device = self.lidar_I2C.scan()
            print(device)

        elif ID==4:
            self.lidar_DIO4.value(0)
            self.lidar_DIO4 = Pin(lidar_DIO, mode=Pin.IN)
            self.write_address(LIDAR4_ADDR)
            self.tof4 = VL53L0X.VL53L0X(i2c=self.lidar_I2C, address=LIDAR4_ADDR)
            device = self.lidar_I2C.scan()
            print(device)

    def initialise_lidar(self, lidar_SDA, lidar_SCL, lidar_DIO1, lidar_DIO2, lidar_DIO3, lidar_DIO4,
                         lidar_baudrate=9600):
        self.lidar_SDA = lidar_SDA
        self.lidar_SCL = lidar_SCL
        self.lidar_baudrate = lidar_baudrate

        self.lidar_DIO1 = Pin(lidar_DIO1, mode=Pin.OUT)
        self.lidar_DIO2 = Pin(lidar_DIO2, mode=Pin.OUT)
        self.lidar_DIO3 = Pin(lidar_DIO3, mode=Pin.OUT)
        self.lidar_DIO4 = Pin(lidar_DIO4, mode=Pin.OUT)
        self.lidar_DIO1.value(0)
        self.lidar_DIO2.value(0)
        self.lidar_DIO3.value(0)
        self.lidar_DIO4.value(0)
        self.lidar_I2C = I2C(2)
        self.lidar_I2C.init(I2C.MASTER, baudrate=self.lidar_baudrate, pins=(self.lidar_SDA, self.lidar_SCL))

        self._set_lidar(ID=1, lidar_DIO=lidar_DIO1)
        self._set_lidar(ID=2, lidar_DIO=lidar_DIO2)
        self._set_lidar(ID=3, lidar_DIO=lidar_DIO3)
        self._set_lidar(ID=4, lidar_DIO=lidar_DIO4)







    def write_address(self, new_addr):
        self.lidar_I2C.writeto_mem(LIDAR_DEFAULT_ADDR, LIDAR_ADDR_REGISTER, new_addr & 0x7F)

    # Function for initialising all pin functionality for the motor driver
    def initialise_motor(self, motor_pin1, motor_pin2, motor_pin3, motor_pin4):
        self.motor_pin1 = motor_pin1
        self.motor_pin2 = motor_pin2
        self.motor_pin3 = motor_pin3
        self.motor_pin4 = motor_pin4
        self.motor_PWM = PWM(0, frequency=500)
        self.duty_cycle = 0.7
        self.motor_stop()

    # Function for initialising all pin functionality for the gyro
    def initialise_gyro(self, gyro_SDA, gyro_SCL, gyro_baudrate=20000):
        self.gyro_SDA = gyro_SDA
        self.gyro_SCL = gyro_SCL
        self.gyro_baudrate = gyro_baudrate

        self.gyro_I2C = I2C(I2C_BUS_0)
        self.gyro_I2C.init(I2C.MASTER, baudrate=self.gyro_baudrate, pins=(self.gyro_SDA, self.gyro_SCL))
        self.gyro = mpu6050.accel(self.gyro_I2C)

    # Function for initialising pin functionality for the temperature sensor
    def initialise_temp(self, temp_SDA, temp_SCL, temp_baudrate=20000):
        self.temp_SDA = temp_SDA
        self.temp_SCL = temp_SCL
        self.temp_baudrate = temp_baudrate
        self.temp_I2C = I2C(I2C_BUS_0)
        self.temp_I2C.init(I2C.MASTER, baudrate=self.temp_baudrate, pins=(self.temp_SDA, self.temp_SCL))
        self.temp_sensor = _tmp102.Tmp102(self.temp_I2C, TEMP_ADDR)



    # Function for stopping all motor function
    def motor_stop(self):
        Pin(self.motor_pin1, mode=Pin.OUT).value(0)  # Pin1
        Pin(self.motor_pin2, mode=Pin.OUT).value(0)  # Pin2
        Pin(self.motor_pin3, mode=Pin.OUT).value(0)  # Pin3
        Pin(self.motor_pin4, mode=Pin.OUT).value(0)  # Pin4

    # Function to commands motor driver forwards, taking in a speed and distance
    def move_forward(self, d, speed):
        self.motor_stop()
        self.motor_PWM.channel(0, pin=self.motor_pin1, duty_cycle=self.duty_cycle)
        self.motor_PWM.channel(0, pin=self.motor_pin3, duty_cycle=self.duty_cycle)
        T = d / speed
        time.sleep(T)
        self.motor_stop()
        return

    # Function to command motor driver backwards, taking in a speed and distance
    def move_backward(self, d, speed):
        self.motor_stop()
        self.motor_PWM.channel(0, pin=self.motor_pin2, duty_cycle=self.duty_cycle)
        self.motor_PWM.channel(0, pin=self.motor_pin4, duty_cycle=self.duty_cycle)
        T = d / speed
        time.sleep(T)
        self.motor_stop()
        return

    # Function to rotate clockwise, taking in an angle and a rotational speed
    def rotate_clockwise(self, angle, rotational_speed):
        self.motor_stop()
        self.motor_PWM.channel(0, pin=self.motor_pin1, duty_cycle=self.duty_cycle)
        self.motor_PWM.channel(0, pin=self.motor_pin4, duty_cycle=self.duty_cycle)
        T = angle / rotational_speed
        time.sleep(T)
        self.motor_stop()
        return

    # Function to rotate anti-clockwise, taking in an angle and a rotational speed
    def rotate_anti_clockwise(self, angle, rotational_speed):
        self.motor_stop()
        self.motor_PWM.channel(0, pin=self.motor_pin2, duty_cycle=self.duty_cycle)
        self.motor_PWM.channel(0, pin=self.motor_pin3, duty_cycle=self.duty_cycle)
        T = angle / rotational_speed
        time.sleep(T)
        self.motor_stop()
        return

    # Function to get lidar readings [TBC]
    def get_lidar(self):
        print("[+] getting lidar")
        self.tof1.start()
        self.tof2.start()
        self.tof3.start()
        self.tof4.start()
        data_tof1 = self.tof1.read()
        data_tof2 = self.tof2.read()
        data_tof3 = self.tof3.read()
        data_tof4 = self.tof4.read()
        print("data_tof1:", data_tof1)
        print("data_tof2:", data_tof2)
        print("data_tof3:", data_tof3)
        print("data_tof4:", data_tof4)
        return data_tof1, data_tof2, data_tof3, data_tof4 #in mm

    #Function to get solar panel voltage throughout the voltage divider
    def get_solar_panel_vol(self):
        vol = self.solar_panel_APIN.value()
        return vol

    # TODO: Write angle calculation funciton
    def get_angle(self):
        # get analog read of gyro minus the zero voltage point
        # Divide this by the gyro sensitivity
        # Add this to the current angle, keep track of global current angle
        return

    # Function to get gyro reading
    def get_gyro(self):
        acc = self.gyro.get_values()
        print("[+] Acc Reading: ", acc)
        return acc

    # Function to get temperature readings
    def get_temp(self):
        temp = self.temp_sensor.temperature
        print("[+] Temp Reading: ", temp)
        return temp

    # Function to test timers for all different sensors
    def test_timer(self):
         self.gyro_timer = Timer.Alarm(self.get_gyro, ms=300, periodic=True)
         self.lidar_timer = Timer.Alarm(self.get_lidar, s=1, periodic=True)
         self.temp_timer = Timer.Alarm(self.get_temp, s=2, periodic=True)

    # Function to test motor functionality
    def test_motor(self):
        self.move_forward(1, 1)
        self.move_backward(1, 1)
        self.rotate_clockwise(90, 2)
        self.rotate_anti_clockwise(90, 2)

    # Function to stop all body functions, movement and sensing
    def stop(self):
        self.gyro_timer.cancel()
        self.lidar_timer.cancel()
        self.temp_timer.cancel()
        self.motor_stop()
        sys.exit()

    def get_pos(self):
        #integrate route2 function -> change name possibly
        angle = self.get_angle()
        l1, l2, l3, l4 = self.get_lidar()
        x, y = self.route2(angle, l1, l2, l3, l4)

        return x, y

    def get_battery_state(self):
        bat = 0.47
        return bat


    def get_state_info(self):
        # Get information, Dummy Variables for now

        temp = self.get_temp()
        x = self.get_x_coord()
        y = self.get_y_coord()
        _time = utime.gmtime()
        battery = self.get_battery_state()
        return temp, x, y, _time, battery


if __name__ == '__main__':
    body = SwarmBody()
    body.get


    print("[+] Setting Timer")
    while True:
        #body.get_lidar()
        body.get_gyro()
        #body.get_temp()
    # body.test_timer()
