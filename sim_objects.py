import numpy as np
from threading import Lock
from helper_tools import *

lock = Lock()
class SimObject:

    def __init__(self,
                 sim_id,
                 initial_state=None,
                 constraints=None,
                 radius_buffer=10,
                 noise=None,
                 object_type="STATIC"):
        self.sim_id = sim_id
        self._command = None
        self.state = [0, 0, 0, 0] \
                if initial_state is None else initial_state
        self.constraints = [10, 2, np.pi/4] \
                if constraints is None else constraints       
        self.radius = radius_buffer
        self.noise = None
        self.object_type = object_type
        assert(type(sim_id) == int, "sim_id must be of type integer")
        assert(len(self.state) == 4, "State must be 4 dimensional")
        assert(len(self.constraints) == 3, "Constraints must be 3 dimensional")

    def update_state(self, delta_t):
        global lock
        lock.acquire()
        # Action commands
        command = self.command
        delta_speed = command[0]
        delta_heading = command[1]

        # Kinematics
        pos_x = self.x
        pos_y = self.y
        speed = self.state[2]
        heading = self.state[3]

        # Update state
        pos_x += speed*np.cos(heading)*delta_t 
        pos_y += speed*np.sin(heading)*delta_t
        heading += delta_heading*delta_t
        speed += delta_speed*delta_t

        # Set state
        self.x = pos_x
        self.y = pos_y
        self.speed = speed
        self.heading = heading
        lock.release()
        self.command = None
        return self.state

    def to_plot_object(self):
        return PlotObject(self.state[0],
                          self.state[1])

    @property
    def state(self):
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
    
    @x.setter
    def x(self, x):
        self.state[0] = x

    @y.setter
    def y(self, y):
        self.state[1] = y

    @speed.setter
    def speed(self, new_speed):
        self.state[2] = new_speed

    @heading.setter
    def heading(self, new_heading):
        self.state[3] = new_heading


    @property
    def command(self):
        command = self._command
        return command

    @command.setter
    def command(self, command):
        if command is None:
            self._command = [0, 0]
            return
        max_delta_speed, max_delta_heading = self.constraints[1::]
        command[0] = clip(command[0], -max_delta_speed, max_delta_speed)
        command[1] = clip(command[1], -max_delta_heading, max_delta_heading)
 
        global lock
        lock.acquire()
        self._command=command
        lock.release()

    def max_speed():
        return self.constraints[0]

    def max_delta_speed():
        return self.constraints[1]

    def max_delta_heading():
        return self.constraints[2]

    def edge_points(self, reference_point):
        return edge_points_of_circle(reference_point,
                                     [self.x, self.y],
                                     self.radius)
    def to_plot_object(self):
        return PlotObject(self.x,
                          self.y,
                          self.heading,
                          [])

class StaticObject(SimObject):
    def __init__(self,
                 sim_id,
                 initial_state,
                 radius_buffer=25):
        
        super.__init__(sim_id,
              initial_state,
              constraints,
              radius_buffer,
              None,
              "STATIC")

    def update_state(self, delta_t):
        return

class BasicUSV(SimObject):
    def __init__(self,
                 sim_id,
                 initial_state=None,
                 constraints=None,
                 radius_buffer=100,
                 noise=None):
    
        super.__init__(sim_id,
              initial_state,
              constraints,
              radius_buffer,
              None,
              "USV")
