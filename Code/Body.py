import pycom
import time
import machine
from machine import Pin
from machine import I2C
from machine import Timer
from micropython import const
from machine import PWM
import mpu6050
import sys

I2C_BUS_0 = const(0)
I2C_BUS_1 = const(1)


class SwarmBody():
    def __init__(self, motor_pin1='P10', motor_pin2='P11', motor_pin3='P12', motor_pin4='P6', lidar_SDA='P9',
                 lidar_SCL='P8',
                 lidar_DIO1='P19', lidar_DIO2='P20', lidar_DIO3='P21', lidar_DIO4='P22', gyro_SDA='P7', gyro_SCL='P5',
                 temp_AD='P13'):
        self.initialise_motor(motor_pin1, motor_pin2, motor_pin3, motor_pin4)
        self.initialise_gyro(gyro_SDA, gyro_SCL)
        self.initialise_lidar(lidar_SDA, lidar_SCL, lidar_DIO1, lidar_DIO2, lidar_DIO3, lidar_DIO4)
        self.initialise_temp(temp_AD)

    # TODO: Solar Panel VD code
    # TODO: Battery Monitor code
    def initialise_lidar(self, lidar_SDA, lidar_SCL, lidar_DIO1, lidar_DIO2, lidar_DIO3, lidar_DIO4,
                         lidar_baudrate=20000):
        self.lidar_SDA = lidar_SDA
        self.lidar_SCL = lidar_SCL
        self.lidar_baudrate = lidar_baudrate
        self.lidar_DIO1 = Pin(lidar_DIO1, mode=Pin.OUT)
        self.lidar_DIO2 = Pin(lidar_DIO2, mode=Pin.OUT)
        self.lidar_DIO3 = Pin(lidar_DIO3, mode=Pin.OUT)
        self.lidar_DIO4 = Pin(lidar_DIO4, mode=Pin.OUT)
        self.lidar_I2C = I2C(I2C_BUS_1, pins=(self.lidar_SDA, self.lidar_SCL))
        self.lidar_I2C.init(I2C.MASTER, baudrate=self.lidar_baudrate)

    def initialise_motor(self, motor_pin1, motor_pin2, motor_pin3, motor_pin4):
        self.motor_pin1 = motor_pin1
        self.motor_pin2 = motor_pin2
        self.motor_pin3 = motor_pin3
        self.motor_pin4 = motor_pin4
        self.motor_PWM = PWM(0, frequency=500)
        self.duty_cycle = 0.7
        self.motor_stop()

    def initialise_gyro(self, gyro_SDA, gyro_SCL, gyro_baudrate=20000):
        self.gyro_SDA = gyro_SDA
        self.gyro_SCL = gyro_SCL
        self.gyro_baudrate = gyro_baudrate
        self.gyro_I2C = I2C(I2C_BUS_0, pins=(self.gyro_SDA, self.gyro_SCL))
        self.gyro_I2C.init(I2C.MASTER, baudrate=self.gyro_baudrate)
        self.gyro = mpu6050.accel(self.gyro_I2C)

    def initialise_temp(self, temp_AD):
        self.temp_ADC = machine.ADC()
        self.temp_AD = self.temp_ADC.channel(pin=temp_AD)

    def motor_stop(self):
        Pin(self.motor_pin1, mode=Pin.OUT).value(0)  # Pin1
        Pin(self.motor_pin2, mode=Pin.OUT).value(0)  # Pin2
        Pin(self.motor_pin3, mode=Pin.OUT).value(0)  # Pin3
        Pin(self.motor_pin4, mode=Pin.OUT).value(0)  # Pin4

    def move_forward(self, d, speed):
        self.motor_stop()
        self.motor_PWM.channel(0, pin=self.motor_pin1, duty_cycle=self.duty_cycle)
        self.motor_PWM.channel(0, pin=self.motor_pin3, duty_cycle=self.duty_cycle)
        T = d / speed
        time.sleep(T)
        self.motor_stop()
        return

    def move_backward(self, d, speed):
        self.motor_stop()
        self.motor_PWM.channel(0, pin=self.motor_pin2, duty_cycle=self.duty_cycle)
        self.motor_PWM.channel(0, pin=self.motor_pin4, duty_cycle=self.duty_cycle)
        T = d / speed
        time.sleep(T)
        self.motor_stop()
        return

    def rotate_clockwise(self, angle, rotational_speed):
        self.motor_stop()
        self.motor_PWM.channel(0, pin=self.motor_pin1, duty_cycle=self.duty_cycle)
        self.motor_PWM.channel(0, pin=self.motor_pin4, duty_cycle=self.duty_cycle)
        T = angle / rotational_speed
        time.sleep(T)
        self.motor_stop()
        return

    def rotate_anti_clockwise(self, angle, rotational_speed):
        self.motor_stop()
        self.motor_PWM.channel(0, pin=self.motor_pin2, duty_cycle=self.duty_cycle)
        self.motor_PWM.channel(0, pin=self.motor_pin3, duty_cycle=self.duty_cycle)
        T = angle / rotational_speed
        time.sleep(T)
        self.motor_stop()
        return

    # TODO: Write lidar function
    def get_lidar(self):
        return

    # TODO: Write angle calculation funciton
    def get_angle(self):
        # get analog read of gyro minus the zero voltage point
        # Divide this by the gyro sensitivity
        # Add this to the current angle, keep track of global current angle
        return

    def get_gyro(self):
        acc = self.gyro.get_values()
        print("[+] Acc Reading: ", acc)
        return acc

    def get_temp(self):
        temp = self.temp_AD()
        print("[+] Temp Reading: ", temp)
        return temp

    def test_timer(self):
        self.gyro_timer = Timer.Alarm(self.get_gyro, ms=300, periodic=True)
        self.lidar_timer = Timer.Alarm(self.get_lidar, s=2, periodic=True)
        self.temp_timer = Timer.Alarm(self.get_temp, s=2, periodic=True)

    def test_motor(self):
        self.move_forward(1, 1)
        self.move_backward(1, 1)
        self.rotate_clockwise(90, 2)
        self.rotate_anti_clockwise(90, 2)

    def stop(self):
        self.gyro_timer.cancel()
        self.lidar_timer.cancel()
        self.temp_timer.cancel()
        self.motor_stop()
        sys.exit()


if __name__ == '__main__':
    body = SwarmBody()

    try:
        body.test_timer()
    except:
        print("[-] Error!")
        print("[-] Exiting Immediately")
        body.stop()
