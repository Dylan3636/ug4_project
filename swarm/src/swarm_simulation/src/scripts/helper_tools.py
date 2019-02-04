from collections import namedtuple
import math

SimState = namedtuple("SimState",  
                      ["x",
                       "y",
                       "speed",
                       "heading",
                       "radius",
                       "object_type"])


def clip(x, min_x, max_x):
    return min(max_x, max(x, min_x))


def center_point(origin, point):
    return tuple([x-o for x, o in zip(point, origin)])


def euclidean_distance(v1, v2):
    return math.sqrt(sum([(x1-x2)**2 for x1, x2 in zip(v1, v2)]))


def mid_point(p1, p2):
    return tuple([(x+y)/2 for x, y in zip(p1, p2)])


def angle_between(p1, p2):
    p3 = center_point(p1, p2)
    print(p3)
    angle = math.atan2(p3[1], p3[0])
    print(angle)
    return angle


def relative_angle_between(p1, p2, offset_angle):
    p3 = center_point(p1, p2)
    angle = math.atan2(p3[1], p3[0])
    if angle < -math.pi/2:
        angle += 2*math.pi
    angle -= offset_angle
    return angle


def less_than_or_close_enough(x1, x2, epsilon=1e-10):
    return x1 < x2 or close_enough(x1, x2, epsilon)


def greater_than_or_close_enough(x1, x2, epsilon=1e-10):
    return x1 > x2 or close_enough(x1, x2, epsilon)


def close_enough(x1, x2, epsilon=1e-10):
    return math.fabs(x1-x2) < epsilon


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
    return leftmost_point, rightmost_point


class Command:
    def __init__(self, delta_speed, delta_heading):
        self.delta_speed = delta_speed
        self.delta_heading = delta_heading
