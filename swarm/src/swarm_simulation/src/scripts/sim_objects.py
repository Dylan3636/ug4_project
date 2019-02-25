import numpy as np
from threading import Lock
from helper_tools import *
from plot import PlotObject


class SimObject:

    def __init__(self,
                 sim_id,
                 initial_state=None,
                 constraints=None,
                 radius_buffer=10,
                 noise=None,
                 object_type="STATIC"):

        self.sim_id = sim_id
        self.command = None
        self._state = [0, 0, 0, 0] if initial_state is None else list(initial_state)
        self.constraints = [10, 2, np.pi/4] \
                if constraints is None else constraints       
        self.radius = radius_buffer
        self.noise = None
        self.object_type = object_type
        self.lock = Lock()
        self.active = False if object_type == "INTRUDER" else True
        assert type(sim_id) == int, "sim_id must be of type integer"
        assert len(self._state) == 4, "State must be 4 dimensional"
        assert len(self.constraints) == 3, "Constraints must be 3 dimensional"

    def to_plot_object(self):
        return PlotObject(self.state[0],
                          self.state[1])

    @property
    def position(self):
        return self.x, self.y

    @property
    def state(self):
        return SimState(self.x, self.y, self.speed, self.heading, self.radius, self.object_type)

    @state.setter
    def state(self, state):
        self._state = state 

    @property
    def x(self):
        return self._state[0]

    @property
    def y(self):
        return self._state[1]

    @property
    def heading(self):
        return self._state[3]

    @property
    def speed(self):
        return self._state[2]
    
    @x.setter
    def x(self, x):
        self._state[0] = x

    @y.setter
    def y(self, y):
        self._state[1] = y

    @speed.setter
    def speed(self, new_speed):
        self._state[2] = new_speed

    @heading.setter
    def heading(self, new_heading):
        while new_heading > 2*np.pi:
            new_heading -= 2*np.pi
        while new_heading < 0:
            new_heading += 2*np.pi
        self._state[3] = new_heading 

    @property
    def command(self):
        command = self._command
        return command

    @command.setter
    def command(self, command):
        if command is None:
            self._command = Command(0, 0)
            return

        max_delta_speed, max_delta_heading = self.constraints[1::]
        command.delta_speed = clip(command.delta_speed,
                                   -max_delta_speed,
                                   max_delta_speed)
 
        command.delta_heading = clip(command.delta_heading,
                                     -max_delta_heading,
                                     max_delta_heading)

        self.lock.acquire()
        self._command = command
        self.lock.release()

    def max_speed(self):
        return self.constraints[0]

    def max_delta_speed(self):
        return self.constraints[1]

    def max_delta_heading(self):
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

    def update_state(self, delta_t):
        self.lock.acquire()
        # Action commands
        command = self.command
        delta_speed = command.delta_speed
        delta_heading = command.delta_heading

        # Kinematics
        pos_x = self.x
        pos_y = self.y
        speed = self.state[2]
        heading = self.state[3]

        # Update state
        heading += delta_heading*delta_t
        speed += delta_speed*delta_t
        pos_x += speed*np.cos(heading)*delta_t
        pos_y += speed*np.sin(heading)*delta_t

        # Set state
        self.x = pos_x
        self.y = pos_y
        self.speed = max(speed, 0)
        self.heading = heading
        self.lock.release()
        self.command = None
        return self.state


class StaticObject(SimObject):
    def __init__(self,
                 sim_id,
                 initial_state,
                 radius_buffer=25):
        
        super().__init__(sim_id,
                         initial_state,
                         None,
                         radius_buffer,
                         None,
                         "STATIC")

    def update_state(self, delta_t):
        return self.state


class Tanker(SimObject):
    def __init__(self,
                 sim_id,
                 initial_state,
                 radius_buffer=25):
        
        super().__init__(sim_id,
                         initial_state,
                         None,
                         radius_buffer,
                         None,
                         "TANKER")


class BasicUSV(SimObject):
    def __init__(self,
                 sim_id,
                 initial_state=None,
                 constraints=None,
                 radius_buffer=100,
                 noise=None):
    
        super().__init__(sim_id,
                         initial_state,
                         constraints,
                         radius_buffer,
                         None,
                         "USV")


class Intruder(SimObject):
    def __init__(self,
                 sim_id,
                 initial_state=None,
                 constraints=None,
                 activate_time=0,
                 radius_buffer=100,
                 noise=None):
        self.activate_time = activate_time
        super().__init__(sim_id,
                         initial_state,
                         constraints,
                         radius_buffer,
                         None,
                         "INTRUDER")
