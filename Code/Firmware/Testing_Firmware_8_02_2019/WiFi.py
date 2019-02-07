"""
+-------------------------------------------------------+
|   This module uses the pycom wifi network library     |
|   and a given config file in form of a nested dict    |
|   to scan available wifi networks and compare whether |
|   they are known networks or not. If they are known,  |
|   then the module connects to that network.           |
+-------------------------------------------------------+
"""

# Importing network to use the WLAN class, importing config to access the known networks, machine and time are standard
# system libraries.
from network import *
from Config import *
import time
import machine
import slogger

slogger.basic_config(level=slogger.NOTSET)
logger = slogger.get_logger("NETWORK")

# Function to return ordered list of tuples for ssid (name of networks)
def signal_order(nets):
    def useRSSI(elem):
        return elem[4]

    return sorted(nets, key=useRSSI, reverse=True)


# WiFi Setup Function that initialises a WLAN() object with certain parameters
def setup():
    wlan = WLAN()
    wlan.mode(WLAN.STA)
    wlan.init(antenna=WLAN.EXT_ANT, power_save=True)
    return wlan


# Wifi connect function, reads available nets and compares to known networks located in Config.py file
# Connects if there is a known net
def connect(known_nets=None):
    wifi = setup()  # Setting up a WLAN object via setup() function defined above
    if wifi.isconnected():  # Disconnect to wifi if already connected
        wifi.disconnect()
    available_nets = signal_order(wifi.scan())  # Ordered list of wifi nets that are available
    if known_nets == None:
        known_nets = config_firmware["wifi"]  # using known configs
    nets_to_try = []
    for elements in available_nets:  # checking if elements in available networks is in our known networks
        if elements.ssid in known_nets:  # adds information of these networks to the nets to try list
            net = known_nets[elements.ssid]
            net['ssid'] = elements.ssid
            net['sec'] = elements.sec
            nets_to_try.append(net)
    print(nets_to_try[0])
    try:
        if (len(nets_to_try) > 0):  # if the networks to try is larger than 0
            for i in range(0, len(nets_to_try)):
                net_to_use = nets_to_try[0]  # Setting variables
                pwd = net_to_use["pwd"]
                sec = net_to_use["sec"]
                wifi.ifconfig(id=0, config='dhcp')
                logger.debug("connect","ssid:{} || sec:{}".format(net_to_use["ssid"], net_to_use["sec"]))
                #print("ssid:", net_to_use['ssid'])
                #print("sec:", net_to_use['sec'])
                if net_to_use["ssid"] == 'VM0134973_2GEXT':  # Forcing connection to home network
                    wifi.connect(net_to_use['ssid'], (sec, pwd), timeout=10000)
                    while not wifi.isconnected():
                        machine.idle()
                    #print("connected to:", net_to_use['ssid'])
                    logger.debug("connect","connected to:{}".format(net_to_use["ssid"]))
                    return True
                elif net_to_use["ssid"] == "Robin":  # Forcing connection to phone hotspot network
                    wifi.connect(net_to_use['ssid'], (sec, pwd), timeout=10000)
                    while not wifi.isconnected():
                        machine.idle()
                    #print("connected to:", net_to_use['ssid'])
                    logger.debug("connected","connected to:{}".format(net_to_use["ssid"]))
                    return True
        else:
            return False
    except Exception as err:
        #print(err)
        #print("failed to connect")
        logger.debug("connect","Failed to connect: {}".format(err))
        return False


if __name__ == '__main__':
    connect()
