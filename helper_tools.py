from collections import namedtuple
import math

Command = namedtuple("Command",
                     ["delta_speed",
                      "delta_heading"])

SimState = namedtuple("SimState",  
                     ["x",
                      "y",
                      "speed",
                      "heading"])

def euclidean_distance(v1, v2):
    return math.sqrt(sum([(x1-x2)**2 for x1,x2 in zip(v1, v2)]))

def edge_points_of_circle(reference_point, center, radius):
    ref_x = reference_point[0]
    ref_y = reference_point[1]
    x = center[0]
    y = center[1]
    r = euclidean_distance([ref_x, ref_y], [x, y])
    alpha = math.atan2(radius, r)
    beta = math.atan2(y-ref_y, x-ref_x)
    hyp = math.sqrt(r**2+radius**2)
    leftmost_point = hyp*math.cos(beta+alpha)+ref_x, hyp*math.sin(beta+alpha)+ref_y
    rightmost_point = hyp*math.cos(beta-alpha)+ref_x, hyp*math.sin(beta-alpha)+ref_y
    return (leftmost_point, rightmost_point)

