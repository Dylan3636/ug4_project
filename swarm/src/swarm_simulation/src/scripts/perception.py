import numpy as np


def perceive(sim_object, sigma=5):
    return sim_object
    tmp = sim_object
    tmp.x = tmp.x + np.random.rand()*sigma
    tmp.y = tmp.y + np.random.rand()*sigma
    return tmp
