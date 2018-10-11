from helper_tools import *
from oa import correct
class USV(SimObject):
    def __init__(self, id, differential_constraints):
        self.id = id
        self.state = [0, 100, 100, 0]
        self.differential_constraints = differential_constraints
        self.object_type = "USV"
    
    def update(self, world_state):
        # obstacles = [[(obj.x, obj.y+20), (obj.x, obj.y-20)] for obj in world_state]
        obstacles = [obj.edge_points([self.x, self.y]) for obj in world_state]
        print(obstacles)
        # xd = obj.x-self.x
        # yd = obj.y-self.y
        # r = np.sqrt((xd)**2+(yd)**2)
        # theta = np.arctan(yd, xd)
        delta_theta = 0-self.heading# np.arctan2(100-self.y, 0)
        command = Command(0, delta_theta)
        differential_constraints= [5, 10]

        print(command)
        try:
            safe_command = correct(self,
                               command,
                               obstacles,
                               differential_constraints,
                               45,
                               1,
                               100,
                               0.999995)
            print(safe_command)
            print(np.rad2deg(safe_command.delta_heading))
        except AssertionError as e:
            print(e)
            safe_command = Command(command[0], max(min(command[1],
                                   np.deg2rad(differential_constraints[1])),
                               -np.deg2rad(differential_constraints[1])))
            print(safe_command)
        self.update_state(safe_command)

    def update_state(self, command):
        self.state[3] += command.delta_heading
        self.state[2] += command.delta_speed
        speed = self.state[2]
        heading = self.state[3]

        self.state[0] += speed*np.cos(heading)*0.1 # TODO self.get_object_by_id(self.id)
        self.state[1] += speed*np.sin(heading)*0.1
        # self.state[3] += np.pi/60


