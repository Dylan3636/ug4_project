import numpy as np
from time import sleep
from plot import LivePlot, PlotObject
from helper_tools import *
from perception import perceive
from planning import avoid
from sim_objects import *


class Simulation:

    def __init__(self, sim_objects, timeout=0.1):
        self.sim_objects = dict([(obj.sim_id, obj) for obj in sim_objects])
        self.OK = True
        self.anim = LivePlot()
        self.timeout=timeout

    def update_simulation_state(self):
        readings = perceive(self.sim_objects)
        for sim_id, obj in self.sim_objects.items():
            if obj.object_type != "STATIC":
                obj.command = avoid(obj, readings)
            obj.update_state(self.timeout)

    def command_by_object_id(self, sim_id, command):
        self.sim_objects[sim_id].command = command
 
    def begin(self):
        while(self.OK):
            self.update_simulation_state()
            self.anim.update_world_state(self.sim_objects.values())
            sleep(self.timeout)

    def kill(self):
        self.OK = False

    @property
    def delta_t(self):
        return self._delta_t

    @delta_t.setter
    def delta_t(self, delta_t):
        self._delta_t = delta_t


def simulation():
    usv_1 = BasicUSV(0, [])
    static_1 = StaticObject(100, [200, 100, 0, 0], 10)
    static_2 = StaticObject(101, [250, 75, 0, 0], 10)


    lp = LivePlot()
    for i in range (1000):
        usv_1.update([static_1, static_2])
        lp.update_world_state(
                [usv_1, static_1, static_2])
    return lp

if __name__ == '__main__':
    usv_1 = BasicUSV(0, [0,75,30,0], radius_buffer=40)
    usv_2 = BasicUSV(1, [300,75,30,np.pi], radius_buffer=40)
    static_1 = StaticObject(100, [200, 100, 0, 0], radius_buffer=30)
    static_2 = StaticObject(101, [250, 75, 0, 0], radius_buffer=30)

    sim=Simulation([usv_1, usv_2, static_1, static_2],
                    0.1)

    sim.begin()
    lp = sim.anim
    lp.canvas.mainloop()
