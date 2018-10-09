import numpy as np
from helper_tools import Command, euclidean_distance


def collision_check(
                    agent_point,
                    leftmost_point,
                    rightmost_point,
                    max_angle=45,
                    step=5,
                    max_distance=100):

    assert max_angle%step == 0, "Max angle is not divisible by step!"
    assert euclidean_distance(agent_point, leftmost_point) < max_distance or euclidean_distance(agent_point, rightmost_point) < max_distance, "Object not close enough to avoid!"
    
    ref_point = np.array(agent_point)
    l_point = np.array(leftmost_point)
    r_point = np.array(rightmost_point)
    
    l_point_centered = (l_point-ref_point)
    r_point_centered = (r_point-ref_point)
    
    l_theta = np.arctan2(l_point_centered[1], l_point_centered[0])
    r_theta = np.arctan2(r_point_centered[1], r_point_centered[0])
    
    print(np.rad2deg(l_theta), np.rad2deg(r_theta))
    # Check if points cover whole region
    if l_theta >= np.deg2rad(max_angle) and r_theta <= -np.deg2rad(max_angle):
        return [True]*int(max_angle/step)*2

    assert angle_check(l_theta, r_theta), "right point is more left than it is right!"
    assert l_theta <= np.deg2rad(max_angle) or r_theta >= -np.deg2rad(max_angle),\
            "left or right point is not within the observable bounds"
    
    blocks = np.arange(-np.deg2rad(max_angle),
                       np.deg2rad(max_angle),
                       np.deg2rad(step))
    # print(np.rad2deg(blocks))
    # print((blocks-l_theta) < 1e-10)
    # print((blocks-r_theta) > -1e-10)
    fan = np.logical_and((blocks - l_theta) < 1e-10,
                    (blocks - r_theta) > -1e-10)
    return fan

def multiple_collision_check(agent_point,
                             obstacle_points,
                             max_angle=45,
                             step=5,
                             max_distance=100):
    fans = []
    for obstacle in obstacle_points:
        try:
            fans.append(collision_check(agent_point,
                                obstacle[0],
                                obstacle[1],
                                max_angle,
                                step,
                                max_distance))
        except AssertionError as e:
            print(e)

    assert len(fans)>0, "failed to return any fan"
    if (len(fans) == 1):
        return fans[0]

    return np.logical_or(*fans)

def angle_check(l_theta, r_theta):
    if l_theta >= 0 and r_theta <=0:
        return True
    else:
        return l_theta >= r_theta

def single_correction(state,
            command,
            leftmost_point,
            rightmost_point,
            differntial_constraints,
            max_theta,
            step):
    fan = collision_check((state.x, state.y),
                          leftmost_point,
                          rightmost_point,
                          max_theta,
                          step)
    return correct_using_fan(state,
                             command,
                             fan,
                             max_theta,
                             step)
def correct(state,
            command,
            points,
            differential_constraints,
            max_theta=45,
            step=5,
            max_distance=100,
            aggression=0.5):

    fan = multiple_collision_check((state.x, state.y),
                          points,
                          max_theta,
                          step,
                          max_distance)

    return correct_using_fan(state,
                             command,
                             fan,
                             differential_constraints,
                             max_theta,
                             step,
                             aggression)

def correct_using_fan(state,
                      command,
                      fan,
                      differential_constraints,
                      max_theta,
                      step,
                      aggression=0.5):

    theta = state.heading
    delta_theta = command.delta_heading
    max_delta_theta = np.deg2rad(differential_constraints[1])
    future_theta = theta + delta_theta
    block = np.deg2rad(np.arange(-max_theta,
                                 max_theta,
                                 step))
    safe_delta_thetas_mask = np.logical_not(fan)
    reachable_delta_thetas_mask = np.abs(block)<max_delta_theta

    safe_and_reachable_mask = np.logical_and(safe_delta_thetas_mask,
                                             reachable_delta_thetas_mask)

    safe_delta_thetas = block[safe_and_reachable_mask]
    print(np.logical_not(fan))
    # print(reachable_delta_thetas_mask)
    print(np.rad2deg(block[safe_delta_thetas_mask]))

    if not list(safe_delta_thetas):
        if future_theta >= 0:
            safe_delta_theta = max_delta_theta
        else:
            safe_delta_theta = -max_delta_theta
    else:
        if np.any(np.abs(safe_delta_thetas-delta_theta)<1e-10):
            safe_delta_theta = delta_theta
        else:
            start=None
            intervals = []
            for i, mark in enumerate(safe_and_reachable_mask):
                if mark: 
                    if start is None:
                        start = i
                    else:
                        if i == np.size(safe_and_reachable_mask)-1:
                            intervals.append((start, i))
                            start=None
                        else:
                            continue
                elif start is not None:
                    intervals.append((start, i-1))
                    start=None
                else:
                    continue
            interval_lenghts = [(block[end]-block[start]) for (start, end) in intervals]
            max_interval_id = np.argmax(interval_lenghts)
            max_interval = intervals[max_interval_id]
            print("INTERVAL")
            print(max_interval)

            if np.abs(block[max_interval[0]]-delta_theta) < np.abs(block[max_interval[1]]-delta_theta):
                safe_delta_theta = aggression*block[max_interval[0]] + (1-aggression)*block[max_interval[1]]
                print(np.rad2deg(block[max_interval[0]]*0.5 + block[max_interval[1]]*0.5))
            else:
                safe_delta_theta = aggression*block[max_interval[1]] + (1-aggression)*block[max_interval[0]]
                print(np.rad2deg(block[max_interval[0]]*0.5 + block[max_interval[1]]*0.5))
            # safe_delta_theta = safe_delta_thetas[np.argmin(np.abs(safe_delta_thetas-delta_theta))]
    print(np.rad2deg(safe_delta_theta))

    command = Command(command[0], safe_delta_theta)
    return command

