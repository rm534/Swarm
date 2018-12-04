from mqtt import MQTTClient
from wifi-connection.py import *
import machine
import time
import ustruct

class Messaging(object):
    def __init__(self):
        return

    def mqtt_connect(self):
        try:
            self.client = MQTTClient("de
