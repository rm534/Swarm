import Body
import Network


class SwarmBehaviour(Body.SwarmBody, Network.SwarmNetwork):
    def __init__(self):
        Body.SwarmBody.__init__(self)
        Network.SwarmNetwork.__init__(self)
        pass

    def get_state(self):
        state = self._get_state_info()

        return state
