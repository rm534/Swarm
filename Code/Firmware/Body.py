import pycom
import time
import utime
import machine
from machine import Pin
from machine import I2C
from machine import Timer
from micropython import const
from machine import PWM
#import Code.Firmware.mpu6050 as
import sys
import Code.Firmware.VL53L0X as VL53L0X
import _thread
# from tmp102 import _tmp102
import math
import Code.Firmware.mpu6050 as mpu6050

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
    def __init__(self, motor_pin1='P10', motor_pin2='P11', motor_pin3='P12',
                 motor_pin4='P6',
                 SDA='P7',
                 SCL='P22',
                 lidar_DIO1='P2', lidar_DIO2='P3', lidar_DIO3='P4', lidar_DIO4='P5'):
        self.initialise_I2C(SDA, SCL)
        # self.initialise_motor(motor_pin1, motor_pin2, motor_pin3, motor_pin4)
        # self.initialise_gyro()
        self.initialise_lidar(SDA, SCL, lidar_DIO1, lidar_DIO2, lidar_DIO3, lidar_DIO4)
        # self.initialise_temp()
        self.gyro_data = 0
        self.robot_move_flag = 0
        self.x = 0
        self.y = 0
        self.temp = 0
        self.battery = 0

    def initialise_solar_panel_monitor(self, solar_panel_AD):
        self.solar_panel_AD = solar_panel_AD
        self.solar_panel_adc = machine.ADC()
        self.solar_panel_APIN = self.solar_panel_adc.channel(pin=self.solar_panel_AD)

    def initialise_I2C(self, SDA, SCL, baudrate=9600):
        self.baudrate = baudrate
        self.SDA = SDA
        self.SCL = SCL
        self.I2C = I2C(2)
        self.I2C.init(I2C.MASTER, baudrate=self.baudrate, pins=(self.SDA, self.SCL))

    # TODO: Battery Monitor code
    # Function for initialising all pin functionality for lidar

    def initialise_lidar(self, SDA, SCL, lidar_DIO1, lidar_DIO2, lidar_DIO3, lidar_DIO4):

        self.lidar_DIO1 = Pin(lidar_DIO1, mode=Pin.OUT)
        self.lidar_DIO2 = Pin(lidar_DIO2, mode=Pin.OUT)
        self.lidar_DIO3 = Pin(lidar_DIO3, mode=Pin.OUT)
        self.lidar_DIO4 = Pin(lidar_DIO4, mode=Pin.OUT)
        self.lidar_DIO1.value(0)
        self.lidar_DIO2.value(0)
        self.lidar_DIO3.value(0)
        self.lidar_DIO4.value(0)

        self._set_lidar(ID=1, lidar_DIO=lidar_DIO1)
        self._set_lidar(ID=2, lidar_DIO=lidar_DIO2)
        self._set_lidar(ID=3, lidar_DIO=lidar_DIO3)
        self._set_lidar(ID=4, lidar_DIO=lidar_DIO4)

    def _set_lidar(self, ID, lidar_DIO):
        if ID == 1:
            self.lidar_DIO1.value(0)
            self.lidar_DIO1 = Pin(lidar_DIO, mode=Pin.IN)
            self.write_address(LIDAR1_ADDR)
            self.tof1 = VL53L0X.VL53L0X(i2c=self.I2C, address=LIDAR1_ADDR)
            device = self.I2C.scan()
            print(device)
        elif ID == 2:
            self.lidar_DIO2.value(0)
            self.lidar_DIO2 = Pin(lidar_DIO, mode=Pin.IN)
            self.write_address(LIDAR2_ADDR)
            self.tof2 = VL53L0X.VL53L0X(i2c=self.I2C, address=LIDAR2_ADDR)
            device = self.I2C.scan()
            print(device)
        elif ID == 3:
            self.lidar_DIO3.value(0)
            self.lidar_DIO3 = Pin(lidar_DIO, mode=Pin.IN)
            self.write_address(LIDAR3_ADDR)
            self.tof3 = VL53L0X.VL53L0X(i2c=self.I2C, address=LIDAR3_ADDR)
            device = self.I2C.scan()
            print(device)

        elif ID == 4:
            self.lidar_DIO4.value(0)
            self.lidar_DIO4 = Pin(lidar_DIO, mode=Pin.IN)
            self.write_address(LIDAR4_ADDR)
            self.tof4 = VL53L0X.VL53L0X(i2c=self.I2C, address=LIDAR4_ADDR)
            device = self.I2C.scan()
            print(device)

    def write_address(self, new_addr):
        self.I2C.writeto_mem(LIDAR_DEFAULT_ADDR, LIDAR_ADDR_REGISTER, new_addr & 0x7F)

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

    def initialise_gyro_new(self):
        self.mpu = mpu6050.MPU6050()
        self.mpu.dmpInitialize()
        self.mpu.setDMPEnabled(True)

        self.packetSize = self.mpu.dmpGetFIFOPacketSize()

        i = 0  # iteration count
        sum_yaw = 0  # summing totals to calculate the mean
        current_yaw = 0  # variable to be updated for every new reading
        fifoCount = self.mpu.getFIFOCount()  #
        while fifoCount < self.packetSize:  # major oscillations without this
            fifoCount = self.mpu.getFIFOCount()  #

        for i in range(0, 600):
            result = self.mpu.getFIFOBytes(self.packetSize)
            q = self.mpu.dmpGetQuaternion(result)
            g = self.mpu.dmpGetGravity(q)
            ypr = self.mpu.dmpGetYawPitchRoll(q, g)
            yaw = ypr['yaw'] * 180 / math.pi

            fifoCount -= self.packetSize
        for i in range(600, 700):
            sum_yaw += yaw
            if i == 700:
                self.avg_yaw = sum_yaw / 100  # avg_yaw = the zero error

        self.alarm_gyro = Timer.Alarm(self._alarm_get_angle, 0.05, periodic=True)

    # Function for initialising pin functionality for the temperature sensor
    def initialise_temp(self):
        self.temp_sensor = _tmp102.Tmp102(self.I2C, TEMP_ADDR)

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
        return data_tof1, data_tof2, data_tof3, data_tof4  # in mm

    # Function to get solar panel voltage throughout the voltage divider
    def get_solar_panel_vol(self):
        vol = self.solar_panel_APIN.value()
        return vol

    # TODO: Write angle calculation function
    def _alarm_get_angle(self, alarm):
        self.get_angle()

    def get_angle(self):
        # get analog read of gyro minus the zero voltage point
        # Divide this by the gyro sensitivity
        # Add this to the current angle, keep track of global current angle

        fifoCount = self.mpu.getFIFOCount()  #
        while fifoCount < self.packetSize:  # major oscillations without this
            fifoCount = self.mpu.getFIFOCount()  #

        result = self.mpu.getFIFOBytes(self.packetSize)
        q = self.mpu.dmpGetQuaternion(result)
        g = self.mpu.dmpGetGravity(q)
        ypr = self.mpu.dmpGetYawPitchRoll(q, g)
        yaw = ypr['yaw'] * 180 / math.pi

        current_yaw = yaw - self.avg_yaw  # ===  [ robot can only start moving at this point ] === #
        self.robot_move_flag = 1
        print(current_yaw)
        self.gyro_data = current_yaw
        return current_yaw

    # Function to get temperature readings
    def get_temp(self):
        temp = self.temp_sensor.temperature
        self.temperature = temp
        print("[+] Temp Reading: ", temp)
        return temp


    # Function to stop all body functions, movement and sensing
    def stop(self):
        self.motor_stop()
        sys.exit()

    # TODO: implement the route method
    def get_pos(self):
        # integrate route2 function -> change name possibly
        angle = self.gyro_data
        l1, l2, l3, l4 = self.get_lidar()
        x, y = Position.route2(angle, l1, l2, l3, l4)
        print("x:{} || y:{}".format(x, y))
        self.x = x
        self.y = y
        return x, y

    def get_battery_state(self):
        bat = 0.47
        self.battery = bat
        return bat

    def _get_state_info(self):
        # Get information, Dummy Variables for now
        return self.temp, self.x, self.y, self.battery

    def _alarm_test(self, alarm):
        self.get_gyro()

    def _alarm_test_pos(self, alarm):
        self.get_pos()

    def set_alarm_gyro_test(self):
        alarm = Timer.Alarm(self._alarm_test, 0.05, periodic=True)
        return alarm

    def set_alarm_pos_test(self, time):
        alarm = Timer.Alarm(self._alarm_test_pos, time, periodic=True)
        return alarm

    def set_alarm_test_thread(self):
        alarm = Timer.Alarm(self.test_thread, 0.05, periodic=True)
        return alarm

    def test_thread(self, alarm):
        thread = _thread.start_new_thread(self.get_gyro)
        thread.exit()


if __name__ == '__main__':
    body = SwarmBody()

    print("[+] Setting Timer")
    while True:
        # body.get_lidar()
        body.get_gyro()
        # body.get_temp()
    # body.test_timer()
