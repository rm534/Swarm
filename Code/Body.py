import pycom
import time
from machine import Pin

tf = 1
tt = 0.25
ts = 0.25


class SwarmBody(motor_1F='P10', motor_1B='P11', motor_2F='P12', motor2B='P9'):
    def __init__(self):
        self.motor_1F = Pin(motor_1F , mode=Pin.OUT)
        self.motor_1B = Pin(motor_1B, mode=Pin.OUT)
        self.motor_2F = Pin(motor_2F, mode=Pin.OUT)
        self.motor_2B = Pin(motor_2B, mode=Pin.OUT)


    #TODO Write the movement functions and design lidar implementation

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
    def get_lidar(self):
        return

