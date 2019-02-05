
import SwarmBot
import Config


VERSION = 0.0
VERSION_DATE = "Nov 2018"
Authors = ["Robin", "Sally", "James", "Ian", "Ben", "Fern", "Nick", "Billy"]
DEVICE_ID = Config.config_firmware["device"]["devid"]


def info():
    print("+------------------------+")
    print("|    SwarmBot            |")
    print("+------------------------+")
    print("| Code v{} {}            |".format(VERSION, VERSION_DATE))
    print("| Device ID: {}          |".format(DEVICE_ID))
    print("+------------------------+")


if __name__ == "__main__":
    swarmbot = SwarmBot.SwarmBot()

    try:
        swarmbot.alive()
        pass
    except:
        print("[-] Error")
        print("[-] Die Immediately")
        swarmbot.die()

