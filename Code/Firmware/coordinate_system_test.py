import Body
import Network
import Network
import _thread
import machine



if __name__ == "__main__":
    body = Body.SwarmBody()
    network = Network.SwarmNetwork()


    #Test Timer with gyro
    #body._set_alarm_test()
    body.initialise_gyro_new()
    alarm_pos = body.set_alarm_pos_test()
    alarm_state = machine.Timer.Alarm(network.send_state_wifi, 20, periodic=True)




    #Test Timer with thread and gyro
    #body._set_alarm_test_thread()
    #_test_alarm_thread(body)


    #TODO: write a test procedure for friday, testing lidar, gyro and coordinate system
    #TODO: draw the algorithm to be tested and reviewed...





