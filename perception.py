import numpy as np

def perceive(sim_objects):
    return [(sim_id, s.x, s.y, s.radius) for sim_id, s in sim_objects.items()]
