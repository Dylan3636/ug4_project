import numpy as np


class OA:
    def __init__(self, differential_constraints):
        self.constraints = differential_constraints
        asset(max_angle%steps == 0)
        self.max_angle = max_angle
        self.steps = steps 

    def occluding(self, reference_point,
                  object_point, left_most_point,
                  right_most_point):
        
        ref_point = np.array(reference_point)
        l_point = np.array(left_most_point)
        r_point = np.array(right_most_point)
        
        l_point_centered = (l_point-ref_point)
        r_point_centered = (r_point-ref_point)
        
        l_theta = np.arctan2(l_point_centered[0], l_point_centered[1])
        r_theta = np.arctan2(r_point_centered[0], r_point_centered[1])
        
        assert(l_theta < r_theta)
        
        blocks = np.arange(-self.max_angle, self.max_angle, steps)
        blocks = np.and((blocks>l_theta), (blocks<r_theta))
        return blocks

    def correct(self, state, command, fan):
        theta = state[3] # state.theta
        delta_theta = command[1] # command.delta_theta
        future_theta = theta + delta_theta

        safe_delta_thetas = (
                np.arange(-self.max_angle,
                          self.max_angle,
                          steps)-theta)[fan]
        if safe_delta_thetas == []:
            #TODO do stuff
            safe_delta_theta = delta
            pass
        else:
            safe_delta_theta = safe_delta_thetas[np.argmin(safe_delta_thetas-delta_theta)]
        command[1] = safe_delta_theta
        return blocks
        
    def 
