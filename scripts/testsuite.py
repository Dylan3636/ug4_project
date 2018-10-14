import unittest
from oa import *
from sim import Command, SimState
from itertools import combinations

class OATests(unittest.TestCase):

    def test_collision_intervals_inside(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]
        max_angle = PI/2
        max_distance = 100
        
        l_thetas = [PI/4, PI/6]
        r_thetas = [-PI/4, -PI/6]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                            points,
                            max_distance,
                            max_angle)

        assert len(intervals) == 1
        interval = intervals.pop()
        self.assertTrue(
                close_enough(interval[0], 
                             PI/4,
                             1e-10) and close_enough(interval[1],
                             -PI/4,
                             1e-10))

    def test_collision_intervals_outside_contained(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]
        max_angle = PI/3
        max_distance = 100
        
        l_thetas = [ PI/6, PI/4]
        r_thetas = [-PI/6, -PI/4]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                            points,
                            max_distance,
                            max_angle)

        assert len(intervals) == 1
        interval = intervals.pop()
        assert close_enough(interval[0], PI/4, 1e-10) and close_enough(interval[1], -PI/4, 1e-10)

    def test_collision_intervals_outside_left(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]
        max_angle = PI/2
        max_distance = 100
        
        l_thetas = [ PI/4, -PI/6]
        r_thetas = [PI/6, -PI/4]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                            points,
                            max_distance,
                            max_angle)

        assert len(intervals) == 2
        interval = intervals.pop(0)
        assert close_enough(interval[0], PI/4, 1e-10) and close_enough(interval[1], PI/6, 1e-10)
        interval = intervals.pop()
        assert close_enough(interval[0], -PI/6, 1e-10) and close_enough(interval[1], -PI/4, 1e-10)

    def test_collision_intervals_outside_right(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]

        max_angle = PI/2
        max_distance = 100
        
        l_thetas = [ -PI/6, PI/4]
        r_thetas = [-PI/4, PI/6]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                                        points,
                                        max_distance,
                                        max_angle)

        assert len(intervals) == 2
        interval = intervals.pop()
        assert close_enough(interval[0], PI/4, 1e-10) and close_enough(interval[1], PI/6, 1e-10)
        interval = intervals.pop()
        assert close_enough(interval[0], -PI/6, 1e-10) and close_enough(interval[1], -PI/4, 1e-10)

    def test_collision_intervals_inbetween_left(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]
        max_angle = PI/4
        max_distance = 100
        
        l_thetas = [ PI/6, PI/4]
        r_thetas = [-PI/4, -PI/6]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                            points,
                            max_distance,
                            max_angle)

        assert len(intervals) == 1
        interval = intervals.pop()
        self.assertTrue(
                close_enough(
                    interval[0],
                    PI/4,
                    1e-10) and close_enough(interval[1], -PI/4, 1e-10))


    def test_collision_intervals_inbetween_right(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]
        max_angle = PI/2
        max_distance = 100
        
        l_thetas = [ PI/4, PI/6]
        r_thetas = [-PI/6, -PI/4]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                            points,
                            max_distance,
                            max_angle)

        self.assertTrue(len(intervals) == 1)
        interval = intervals.pop()
        self.assertTrue(close_enough(
            interval[0],
            PI/4, 1e-10) and close_enough(interval[1],
            -PI/4,
            1e-10))

    def test_safe_intervals_outside_left(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]
        max_angle = PI/4
        max_distance = 100
        
        l_thetas = [ PI/4, -PI/6]
        r_thetas = [PI/6, -PI/4]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                            points,
                            max_distance,
                            max_angle)
        intervals = safe_intervals(intervals, max_angle)
        print(np.rad2deg(intervals))

        assert len(intervals) == 1
        interval = intervals.pop(0)
        assert close_enough(interval[0], PI/6, 1e-10) and close_enough(interval[1], -PI/6, 1e-10)
        # interval = intervals.pop()
        # assert close_enough(interval[0], -PI/6, 1e-10) and close_enough(interval[1], -PI/4, 1e-10)

    def test_safe_intervals_outside_right(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]
        max_angle = PI/4
        max_distance = 100
        max_angle = PI/4
        max_distance = 100
        
        l_thetas = [-PI/6, PI/4]
        r_thetas = [-PI/4, PI/6]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                            points,
                            max_distance,
                            max_angle)
        intervals = safe_intervals(intervals, max_angle)
        print(np.rad2deg(intervals))

        assert len(intervals) == 1
        interval = intervals.pop(0)
        assert close_enough(interval[0], PI/6, 1e-10) and close_enough(interval[1], -PI/6, 1e-10)
 
    def test_safe_intervals_with_wider_fan(self):
        x=10
        y=20
        speed=0
        heading=0
        agent_state = [x, y, speed, heading]
        max_angle = PI/2
        max_distance = 100
        
        l_thetas = [-PI/6, PI/4]
        r_thetas = [-PI/4, PI/6]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))

        intervals = collision_intervals(agent_state,
                            points,
                            max_distance,
                            max_angle)
        intervals = safe_intervals(intervals, max_angle)

        assert len(intervals) == 3
        interval = intervals.pop(0)
        assert close_enough(interval[0], PI/2, 1e-10) and close_enough(interval[1], PI/4, 1e-10)
        interval = intervals.pop(0)
        assert close_enough(interval[0], -PI/4, 1e-10) and close_enough(interval[1], -PI/2, 1e-10)
        interval = intervals.pop(0)
        assert close_enough(interval[0], PI/6, 1e-10) and close_enough(interval[1], -PI/6, 1e-10)
 

    def test_collision_check(self):
        max_angle = np.deg2rad(90)
        agent_point = [10, 20]
        step=5
        angle_combos = np.arange(-max_angle, max_angle, step)
        combos = list(combinations(angle_combos, 2))
        for combo in combos:
            r=10
            leftmost_point  = (
                    10*np.cos(combo[1])+agent_point[0],
                    10*np.sin(combo[1])+agent_point[1])

            rightmost_point = (
                    10*np.cos(combo[0])+agent_point[0],
                    10*np.sin(combo[0])+agent_point[1])

            interval = collision_check(agent_point,
                                       leftmost_point,
                                       rightmost_point,
                                       100,
                                       PI/4)
            
            flag = combo[0]<max_angle or combo[1]>-max_angle
            self.assertFalse(interval is None and flag )
            self.assertTrue(interval is not None and flag )
    
    def test_left_greater_than_right_assertion(self):
        # Check for error if l_theta >= r_theta
        x=10
        y=20
        agent_pos = [x, y]
        max_angle = PI/2
        max_distance = 100
        
        l_thetas = [-PI/4]
        r_thetas = [PI/6]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))


        with self.assertRaises(AssertionError):
            collision_check([0, 0, 0, 0],
                            [1, -1],
                            [1, 1],
                            100,
                            max_angle=PI/2,
                            )
    def test_collision_angles_are_clipped(self):
        # Check for error if l_theta >= r_theta
        x=10
        y=20
        agent_pos = [x, y]
        max_angle = PI/12
        max_distance = 100
        
        l_thetas = [-PI/4]
        r_thetas = [PI/6]

        l_points = [(np.cos(l_theta) + x, np.sin(l_theta) + y) for l_theta in l_thetas]
        r_points = [(np.cos(r_theta) + x, np.sin(r_theta) + y) for r_theta in r_thetas]
        points = list(zip(l_points, r_points))


        interval = collision_check([0, 0, 0, 0],
                            [0, 1],
                            [0, -1],
                            100,
                            max_angle=max_angle,
                            )
        self.assertFalse(interval is None)

        self.assertTrue(close_enough(
            interval[0],
            PI/12, 1e-10) and close_enough(interval[1],
            -PI/12,
            1e-10))

    def test_collision_assertion_3(self):
        return
        # Check for error if l_theta > -max_angle or r_theta > max_angle 
        with self.assertRaises(AssertionError):
            collision_check([0, 0],
                            [-1, 0],
                            [1, 0],
                            max_angle=45,
                            step=5
                            )

    def test_correction(self):
        max_distance = 100
        max_theta = PI/4
        step = 5
        r=10

        command = Command(5, np.pi/6)
        l_theta = np.deg2rad(35)
        r_theta = np.deg2rad(25)

        leftmost_point = (r*np.cos(l_theta)+10, r*np.sin(l_theta)+20)
        rightmost_point = (r*np.cos(r_theta)+10, r*np.sin(r_theta)+20)
        state = SimState(10, 20, 0, 0)

        safe_command = correct(state,
                               command,
                               [(leftmost_point,
                               rightmost_point)],
                               [10, 2, np.pi/4],
                               max_distance,
                               max_theta,
                               aggression=1)

        self.assertTrue(np.abs(safe_command.delta_heading-np.deg2rad(25))<1e-10)


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(OATests)
    unittest.TextTestRunner(verbosity=4).run(suite)
