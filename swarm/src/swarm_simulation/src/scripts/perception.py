import numpy as np
from helper_tools import SimState


def perceive(sim_object, sigma=5):
    # return sim_object
    x, y, speed, heading, radius, obj_type = sim_object

    x += np.random.rand()*sigma
    y += np.random.rand()*sigma

    return SimState(x, y, speed, heading, radius, obj_type)
