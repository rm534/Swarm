import Body
import Network

class SwarmBehaviour(Body.SwarmBody, Network.SwarmNetwork):
    def __init__(self):
        pass


    def get_state(self):
        state = self.get_state_info()
        return state
