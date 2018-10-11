import numpy as np
from threading import Lock
from time import sleep
from plot import LivePlot, PlotObject
from helper_tools import *

lock = Lock()

class SimObject:

    def __init__(self, sim_id, , initial_state, , constraints=None):
        self.command_buffer = None
        self.constraints = None
    
    def update_state(self):
        global lock
        lock.acquire()
        # Action commands
        command = self.command
        delta_speed = command[0]
        delta_heading = command[1]

        # Kinematics
        delta_t = self.delta_t
        pos_x = self.x
        pos_y = self.y
        speed = self.state[2]
        heading = self.state[3]

        # Update state
        heading += delta_heading*delta_t
        speed += delta_speed*delta_t
        pos_x += speed*np.cos(heading)*0.1 
        pos_y += speed*np.sin(heading)*0.1

        # Set state
        self.x = pos_x
        self.y = pos_y
        self.speed = speed
        self.heading = heading
        lock.release()
        return self.state

    def to_plot_object(self):
        return PlotObject(self.state[0],
                          self.state[1])

    @property
    def state():
        return self._state

    @state.setter
    def state(self, state):
        self._state = state

    @property
    def x(self):
        return self.state[0]

    @property
    def y(self):
        return self.state[1]

    @property
    def heading(self):
        return self.state[3]

    @property
    def speed(self):
        return self.state[2]

    @heading.setter
    def heading(self, new_heading):
        self._state[3] = new_heading

    @speed.setter
    def speed(self, new_speed):
        self._state[2] = new_speed

    @property
    def delta_t(self):
        return self._delta_t

    @delta_t.setter
    def delta_t(self, delta_t):
        self._delta_t = delta_t

    @property
    def command(self):
        global lock
        lock.acquire()
        command = self.command_buffer
        lock.release()
        return command

    @command.setter
    def command(self, command)
        max_delta_speed, max_delta_heading = self.constraints[1::]
        command[0] = clip(command[0], -max_delta_speed, max_delta_speed)
        command[1] = clip(command[1], -max_delta_heading, max_delta_heading)
 
        global lock
        lock.acquire()
        self.command_buffer=command
        lock.release()


    def to_plot_object(self):
        return PlotObject(self.x,
                          self.y,
                          self.heading,
                          [])

class StaticObject(SimObject):
    def __init__(self, id, initial_state, radius=25):
        self.id = id
        self.state = initial_state
        self.object_type = "STATIC"
        self.radius = radius

    def update(self, world_space):
        return

    def edge_points(self, reference_point):
        return edge_points_of_circle(reference_point,
                                     [self.x, self.y],
                                     self.radius)

class Simulation:

    def __init__(self, sim_objects, timeout=0.1):
        self.sim_objects = dict([(obj.sim_id, obj) for obj in sim_objects])
        self.OK = True

    def update_simulation_state():
        for obj in self.sim_objects:
            state = obj.update_state()

    def command_by_object_id(self, sim_id, command):
        self.sim_objects[sim_id].command = command
 
    def simulate(self, timeout):
        while(self.OK):
            self.update_simulation_state()
            sleep(timeout)

    def kill(self):
        self.OK = False



def simulation():
    usv_1 = USV(0, [])
    static_1 = StaticObject(100, [200, 100, 0, 0], 10)
    static_2 = StaticObject(101, [250, 75, 0, 0], 10)


    lp = LivePlot()
    for i in range (1000):
        usv_1.update([static_1, static_2])
        lp.update_world_state(
                [usv_1, static_1, static_2])
    return lp

if __name__ == '__main__':
    lp = simulation()
    lp.canvas.mainloop()
