import numpy as np
from time import sleep
from plot import LivePlot, PlotObject
class SimObject:

    def __init__(self):
        pass
    
    def update(self, world_state):
        pass

    def to_plot_object(self):
        return PlotObject(self.state[0],
                          self.state[1])

    @property
    def state():
        return self.state

    @state.setter
    def state(self, state):
        self.inner_state = state

    @state.getter
    def state(self):
        return self.inner_state

    @property
    def x(self):
        return self.state[0]

    @property
    def y(self):
        return self.state[1]

    def to_plot_object(self):
        return PlotObject(self.x,
                          self.y)

class StaticObject(SimObject):
    def __init__(self, id, initial_state):
        self.id = id
        self.state = initial_state

    def update(self, world_space):
        return

class USV(SimObject):
    def __init__(self, id, differential_constraints):
        self.id = id
        self.state = [0, 0, 1, 0]
        self.differential_constraints = differential_constraints
    
    def update(self, world_state):
        speed, heading = self.state[2::]
        self.state[0] += speed*np.cos(heading)*0.1 # TODO self.get_object_by_id(self.id)
        self.state[1] += speed*np.sin(heading)*0.1

    
def simulation():
    usv_1 = USV(0, [])
    static_1 = StaticObject(100, [5, 0, 0, 0])

    lp = LivePlot()
    for i in range (1000):
        usv_1.update(None)
        lp.update_world_state(
                [static_1, usv_1.to_plot_object()])

if __name__ == '__main__':
    simulation()
