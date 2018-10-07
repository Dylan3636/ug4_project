import numpy as np

class ObstacleAvoidanceFan:
    def __init__(self, max_angle, steps):
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
