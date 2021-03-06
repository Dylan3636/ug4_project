import numpy as np
from oa import *
from helper_tools import *

def avoid(sim_obj, readings):
    edges = [edge_points_of_circle([sim_obj.x, sim_obj.y], [reading[1], reading[2]], reading[3]) for reading in readings if reading[0] != sim_obj.sim_id  ]

    print('USV: ', sim_obj.sim_id)
    print('Heading: ', np.rad2deg(sim_obj.heading))
    if np.pi/2 > sim_obj.heading:
        delta_theta = -sim_obj.heading
    elif np.pi> sim_obj.heading >= np.pi/2:
        delta_theta = -(np.pi - sim_obj.heading)
    elif (3/2)*np.pi> sim_obj.heading >= np.pi:
        delta_theta = (np.pi - sim_obj.heading)
    else:
        delta_theta = -(2*np.pi-sim_obj.heading)
    # print(np.rad2deg(delta_theta))
    command = Command(0, delta_theta)
    differential_constraints= [10, 2, np.pi/6]
    print(edges)

    try:
        safe_command = correct(sim_obj,
                           command,
                           edges,
                           differential_constraints,
                           1000,
                           np.pi/2,
                           0.9)
    except AssertionError as e:
        raise(e)
        safe_command = Command(command.delta_speed, max(min(command.delta_heading,
                               np.deg2rad(differential_constraints[1])),
                           -np.deg2rad(differential_constraints[1])))
    return safe_command
