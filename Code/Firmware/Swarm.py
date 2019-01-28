import Body
import Network
import Behaviour

class Swarm(object):
    def __init__(self):
        self.body = Body.SwarmBody()
        self.network = Network.SwarmNetwork()

