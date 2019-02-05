import Body
import Network
import Behaviour


class SwarmBot(Behaviour.SwarmBehaviour):
    def alive(self):
        #get initial position
        #generate blank maps
        #choose target square
        #move towards destination
        #if arrived -> choose target...

        self.get_state_info()
        self.send_state_wifi()

    def die(self):
        pass

    def test(self):
        pass
