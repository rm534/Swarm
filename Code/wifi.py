#STATUS: Tested for home network -> can connect
from network import *
from config import *
import time
import machine

#Function to return ordered list of tuples for ssid
def signal_order(nets):
    def useRSSI(elem):
        return elem[4]
    return sorted(nets, key=useRSSI, reverse=True)

#WiFi Setup Function
def setup():
    wlan = WLAN()
    wlan.mode(WLAN.STA)
    wlan.init(antenna=WLAN.EXT_ANT, power_save=True)
    return wlan

#Wifi connect function, reads available nets and compares to knowns nets
#Connects if there is a known net
def connect(known_nets=None):
    wifi = setup()
    if wifi.isconnected():
        wifi.disconnect()
    available_nets = signal_order(wifi.scan())
    if known_nets == None:
        known_nets = config_firmware["wifi"]
    nets_to_try = []
    for elements in available_nets:
        if elements.ssid in known_nets:
            net = known_nets[elements.ssid]
            net['ssid'] = elements.ssid
            net['sec'] = elements.sec
            nets_to_try.append(net)
    try:
        if (len(nets_to_try) > 0):
            for i in range (0, len(nets_to_try)):
                net_to_use = nets_to_try[0]
                user = net_to_use["user"]
                pwd = net_to_use["pwd"]
                sec = net_to_use["sec"]
                wifi.ifconfig(id=0, config='dhcp')
                print("ssid:", net_to_use['ssid'])
                print("sec:", net_to_use['sec'])
                if net_to_use["ssid"] == 'VM0134973_2GEXT':
                    wifi.connect(net_to_use['ssid'], (sec, pwd), timeout=10000)
                    while not wifi.isconnected():
                        machine.idle()
                    print("connected to:", net_to_use['ssid'])
                elif sec == WLAN.WPA2_ENT:
                    wifi.connect(ssid=net_to_use['ssid'], auth=(sec, user, pwd), identity='rob', ca_certs='none')
                return True
        else:
            return False
    except Exception as err:
        print(err)
        print("failed to connect")
        return False



if __name__ == '__main__':
    connect()
