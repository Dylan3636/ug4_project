import numpy as np
from oa import *
from helper_tools import *

def avoid(sim_obj, readings):
    edges = [edge_points_of_circle([sim_obj.x, sim_obj.y], [reading[1], reading[2]], reading[3]) for reading in readings if reading[0] != sim_obj.sim_id  ]

    # print(np.rad2deg(sim_obj.heading))
    delta_theta = (2*np.pi-sim_obj.heading) if sim_obj.heading > np.pi else -sim_obj.heading   # np.arctan2(100-self.y, 0)
    # print(np.rad2deg(delta_theta))
    command = Command(0, delta_theta)
    differential_constraints= [10, 2, np.pi/18]
    # print(edges)

    try:
        safe_command = correct(sim_obj,
                           command,
                           edges,
                           differential_constraints,
                           500,
                           np.pi/4,
                           0.99)
    except AssertionError as e:
        raise(e)
        safe_command = Command(command.delta_speed, max(min(command.delta_heading,
                               np.deg2rad(differential_constraints[1])),
                           -np.deg2rad(differential_constraints[1])))
    return safe_command
