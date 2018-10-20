import numpy as np
from numpy import arctan2
from numpy import pi as PI
from helper_tools import *


def collision_check(agent_state,
                    left_edge,
                    right_edge,
                    max_distance=45,
                    max_angle=PI/4):
    # Ensure agent doesn't try to look behind itself
    assert max_angle <= PI/2, "{} </= PI/2".rad2deg(max_angle)

    agent_position = agent_state[0:2]
    agent_heading = agent_state[3]

    distance = euclidean_distance(agent_position,mid_point(left_edge,
                                                           right_edge))
    l_theta = relative_angle_between(agent_state[0:2], left_edge, agent_heading)
    r_theta = relative_angle_between(agent_state[0:2], right_edge, agent_heading)

    # Check if it's in the observation fan
    flag = in_fan(distance, l_theta, r_theta, max_distance, max_angle)

    if flag:
        # To be between visible bounds
        return (min(l_theta, max_angle), max(r_theta, -max_angle))
    else:
        return None

def in_fan(distance, l_theta, r_theta, max_distance, max_angle):
    if not(PI/2 >= l_theta >= -PI/2 and PI/2 >= r_theta >= -PI/2):
        # Ignores objects behind it.
        return False
    assert angle_check(l_theta, r_theta), "right point at {} degrees is more left than the left point at {} degrees!".format(np.rad2deg(l_theta), np.rad2deg(r_theta))
    flag = not(l_theta < -max_angle or r_theta > max_angle)
    flag = distance<max_distance and flag
    return flag

def angle_check(l_theta, r_theta):
    if l_theta >= 0 and r_theta <=0:
        return True
    else:
        return l_theta >= r_theta

def safe_intervals(occupied_intervals, max_angle):
    if np.size(occupied_intervals, 0) == 0:
        return [[max_angle, -max_angle]]
    if np.size(occupied_intervals, 0) == 1:
        interval = occupied_intervals[0]
        if interval[0] == max_angle and interval[1] == -max_angle:
            return []
    occupied_intervals = np.sort(occupied_intervals, 0)[::-1, :]
    safe_intervals = []

    if not close_enough(occupied_intervals[0][0], max_angle):
        safe_intervals.append((max_angle, occupied_intervals[0][0]))

    if not close_enough(occupied_intervals[-1][1], -max_angle):
        safe_intervals.append((occupied_intervals[-1][1], -max_angle))

    rp = None
    for interval in occupied_intervals:
        if rp is None:
            rp = interval[1]
        else:
            safe_intervals.append((rp, interval[0]))
    return safe_intervals

def correct(state,
            command,
            points,
            differential_constraints,
            max_distance=100,
            max_angle=PI/4,
            aggression=0.5):

    agent_pos = [state.x, state.y]
    max_delta_heading = differential_constraints[2]
    occupied_intervals = collision_intervals(state.state,
                                    points,
                                    max_distance,
                                    max_angle
                                    )

    intervals = safe_intervals(occupied_intervals, max_angle)
    print("Occupied intervals: ",np.rad2deg(occupied_intervals))
    print("Safe intervals: ", np.rad2deg(intervals))

    if not intervals:
        safe_command = Command(command.delta_speed, -max_delta_heading)
        return safe_command

    delta_heading = clip(command.delta_heading, -max_delta_heading, max_delta_heading)
    print("Requested command: ", np.rad2deg(command.delta_heading))
    print("Requested clipped command: ", np.rad2deg(delta_heading))
    command.delta_heading = delta_heading

    intervals = np.array(intervals)
    if np.any(np.logical_and(intervals[:, 0] >= delta_heading,
                             intervals[:, 1] <= delta_heading)):
        print("Already safe!")
        return command
    
    diff = intervals[:, 0]-intervals[:, 1]
    max_id = np.argmax(diff)
    largest_safe_interval = intervals[max_id, :]
    l_theta, r_theta = largest_safe_interval

    if abs(l_theta-delta_heading)<abs(r_theta-delta_heading):
        safe_delta_heading = aggression*l_theta + (1-aggression)*r_theta
    else:
        safe_delta_heading = aggression*r_theta + (1-aggression)*l_theta

    safe_delta_heading = clip(safe_delta_heading, -max_delta_heading, max_delta_heading)
    safe_command = Command(command.delta_speed, safe_delta_heading)
    print("Safe command: ", np.rad2deg(safe_delta_heading))
    return safe_command

def collision_intervals(agent_state,
                        points,
                        max_distance,
                        max_angle
                        ):

    intervals=[]
    for point in points:
        left_edge, right_edge = point
        interval = collision_check(agent_state,
                                   left_edge,
                                   right_edge,
                                   max_distance,
                                   max_angle
                                   )
        if interval is None:
            continue
        l_theta, r_theta = interval
        contained = False
        for interval in intervals:
            left, right = interval
            if l_theta <= left:
                if r_theta >= right:
                    contained = True
                    break
                else:
                    if l_theta >= right:
                        contained=True
                        interval[1] = r_theta
                    else:
                        contained=False
                        continue
            elif r_theta >= right:
                if r_theta <= left:
                    contained=True
                    interval[0] = l_theta
                else:
                    contained=False
                    continue
            else:
                contained = True
                interval[0] = l_theta
                interval[1] = r_theta

        if not contained:
            intervals.append([l_theta, r_theta])
    return intervals

