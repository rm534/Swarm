import pycom
import time
from machine import Pin

pycom.heartbeat(True)

tf = 1
tt = 0.25
ts = 0.25

#Motor 1
Motor1F = Pin('P10', mode=Pin.OUT)
Motor1B = Pin('P11', mode=Pin.OUT)

#Motor 2
Motor2F = Pin('P12', mode=Pin.OUT)
Motor2B = Pin('P9', mode=Pin.OUT)


def forward():
    Motor1F.value(1)
    Motor2F.value(1)

    time.sleep(tf)
    Motor1F.value(0)
    Motor2F.value(0)

def back():
    Motor1B.value(1)
    Motor2B.value(1)

    time.sleep(tf)
    Motor1B.value(0)
    Motor2B.value(0)

def spin_clock():
    Motor1F.value(1)
    Motor2F.value(0)
    Motor1B.value(0)
    Motor2B.value(1)

    time.sleep(ts)
    Motor1F.value(0)
    Motor2F.value(0)
    Motor1B.value(0)
    Motor2B.value(0)

def spin_anticlock():
    Motor1F.value(0)
    Motor2F.value(1)
    Motor1B.value(1)
    Motor2B.value(0)

    time.sleep(ts)
    Motor1F.value(0)
    Motor2F.value(0)
    Motor1B.value(0)
    Motor2B.value(0)

def motor_1_forward():
    Motor1F.value(1)
    Motor1B.value(0)

    time.sleep(tt)
    Motor1F.value(0)

def motor_1_back():
    Motor1F.value(0)
    Motor1B.value(1)

    time.sleep(tt)
    Motor1B.value(0)

def motor_2_forward():
    Motor2F.value(1)
    Motor2B.value(0)

    time.sleep(tt)
    Motor2F.value(0)

def motor_2_back():
    Motor2F.value(0)
    Motor2B.value(1)

    time.sleep(tt)
    Motor2B.value(0)

def stop():
    Motor1F.value(0)
    Motor2F.value(0)
    Motor1B.value(0)
    Motor2B.value(0)

def commands():
    print('\n')
    print("w/s => Left motor forwards/backwards.")
    print("o/l => Right motor forwards/backwards.")
    print("t/g => All motors forwards/backwards.")
    print("q => Stop all motors.")
    print('\n')
    print("c => Print commands. ")
    print("x => Exit program.")

commands()

while True:

    char = input('\n'"Enter Command Here: ")

    print("Please Stand By.")

    if(char == "w"):
        motor_1_forward()
        continue

    if(char == "s"):
        motor_1_back()
        continue

    if(char == "o"):
        motor_2_forward()
        continue

    if(char == "l"):
        motor_2_back()
        continue

    if(char == "t"):
        forward()
        continue

    if(char == "g"):
        back()
        continue

    if(char == "y"):
        spin_anticlock()
        continue

    if(char == "r"):
        spin_clock()
        continue

    if(char == "x"):
        print("Program Ended.")
        continue

    if(char == "q"):
        print("All motors stopped.")
        stop()
        continue

    if(char == "c"):
        commands()
        continue

    else:
        print("Invalid input !!")
        commands()

## End of Script ##
