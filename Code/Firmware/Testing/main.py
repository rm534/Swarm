import Code.Firmware.Body as Body
import Code.Firmware.Network as Network

if __name__ == "__main__":

    body = Body.SwarmBody()
    network = Network.SwarmNetwork()

    body.initialise_gyro_new()
    body.set_alarm_pos_test(time=3)
    network.alarm_send_state(time=100)
