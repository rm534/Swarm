import pycom
import time
from machine import Pin
from machine import I2C
from machine import Timer
import mpu6050


class SwarmBody():
    def __init__(self, motor_1F='P5', motor_1B='P6', motor_2F='P7', motor_2B='P8', lidar_SDA='P9', lidar_SCL='P10',
                 lidar_DIO1='P19', lidar_DIO2='P20', lidar_DIO3='P21', lidar_DIO4='P22', gyro_SDA='P11', gyro_SCL='P12',
                 temp_AD='P13'):
        self.initialise_motor(motor_1F, motor_1B, motor_2F, motor_2B)
        self.initialise_gyro(gyro_SDA, gyro_SCL)
        self.initialise_lidar(lidar_SDA, lidar_SCL, lidar_DIO1, lidar_DIO2, lidar_DIO3, lidar_DIO4)

    # TODO: initialise temperature sensor
    def initialise_lidar(self, lidar_SDA, lidar_SCL, lidar_DIO1, lidar_DIO2, lidar_DIO3, lidar_DIO4,
                         lidar_baudrate=20000):
        self.lidar_SDA = lidar_SDA
        self.lidar_SCL = lidar_SCL
        self.lidar_baudrate = lidar_baudrate
        self.lidar_DIO1 = Pin(lidar_DIO1, mode=Pin.OUT)
        self.lidar_DIO2 = Pin(lidar_DIO2, mode=Pin.OUT)
        self.lidar_DIO3 = Pin(lidar_DIO3, mode=Pin.OUT)
        self.lidar_DIO4 = Pin(lidar_DIO4, mode=Pin.OUT)
        self.lidar_I2C = I2C(1, pins=(self.lidar_SDA, self.lidar_SCL))
        self.lidar_I2C.init(I2C.MASTER, lidar_baudrate)

    def initialise_motor(self, motor_1F, motor_1B, motor_2F, motor_2B):
        self.motor_1F = Pin(motor_1F, mode=Pin.OUT)
        self.motor_1B = Pin(motor_1B, mode=Pin.OUT)
        self.motor_2F = Pin(motor_2F, mode=Pin.OUT)
        self.motor_2B = Pin(motor_2B, mode=Pin.OUT)

    def initialise_gyro(self, gyro_SDA, gyro_SCL, gyro_baudrate=20000):
        self.gyro_SDA = gyro_SDA
        self.gyro_SCL = gyro_SCL
        self.gyro_I2C = I2C(0, pins=(self.gyro_SDA, self.gyro_SCL))
        self.gyro_I2C.init(I2C.MASTER, baudrate=gyro_baudrate)
        self.gyro = mpu6050.accel(self.gyro_I2C)

    def move_forward(self, dt):
        self.motor_1F.value(1)
        self.motor_2F.value(1)
        time.sleep(dt)
        self.motor_1F.value(0)
        self.motor_2F.value(0)
        return

    def move_backward(self, dt):
        self.motor_1B.value(1)
        self.motor_2B.value(1)
        time.sleep(dt)
        self.motor_1B.value(0)
        self.motor_2B.value(0)
        return

    def move_right(self):
        return

    def move_left(self):
        return

    # TODO Write the movement functions and design lidar implementation
    def get_lidar(self):
        return

    def get_angle(self):
        # get analog read of gyro minus the zero voltage point
        # Divide this by the gyro sensitivity
        # Add this to the current angle, keep track of global current angle
        return

    def get_gyro(self):
        acc = self.gyro.get_values()
        print("[+] Acc Reading: ", acc)
        return acc

    def test_timer(self):
        gyro_timer = Timer.Alarm(self.get_gyro, s=2, periodic=True)
        lidar_timer = Timer.Alarm(self.get_lidar, s=2, periodic=True)

    def test_motor(self):
        self.move_forward(1)
        self.move_backward(1)


if __name__ == '__main__':
    body = SwarmBody()
    body.test_timer()
