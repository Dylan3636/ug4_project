import unittest
from oa import *
from sim import Command, SimState
from itertools import combinations

class OATests(unittest.TestCase):
    def test_occlusion(self):
        max_angle = np.deg2rad(45)
        step = np.deg2rad(5)
        agent_point = [10, 20]
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

            angle_map = collision_check(agent_point, leftmost_point, rightmost_point, 45, 5)
            n_l = int(combo[0]/step + np.size(angle_combos)/2)
            n_r = int(combo[1]/step + np.size(angle_combos)/2)

            print(n_l, n_r)
            print(angle_map[n_l:n_r+1])
            print(angle_map)
            self.assertTrue(np.all(angle_map[n_l:n_r+1]))
            self.assertFalse(np.any(angle_map[0:n_l]) or np.any(angle_map[n_r+1::]))
    
    def test_collision_assertion_1(self):
        # Check for error if l_theta > r_theta
        with self.assertRaises(AssertionError):
            collision_check([0, 0],
                            [1, -1],
                            [1, 1],
                            max_angle=45,
                            step=5
                            )

    def test_collision_assertion_2(self):
        # Check for error if max_angle % step != 0
        with self.assertRaises(AssertionError):
            collision_check([0, 0],
                            [-1, 0],
                            [1, 0],
                            max_angle=45,
                            step=6
                            )

    def test_collision_assertion_3(self):
        # Check for error if l_theta > -max_angle or r_theta > max_angle 
        with self.assertRaises(AssertionError):
            collision_check([0, 0],
                            [-1, 0],
                            [1, 0],
                            max_angle=45,
                            step=5
                            )
    def test_multiple_collision_check(self):
        max_angle = np.deg2rad(45)
        step = np.deg2rad(5)
        agent_point = [10, 20]
        angle_combos = np.arange(-max_angle, max_angle, step)
        combos = [np.deg2rad([-45, -40]), np.deg2rad([35, 40])] # list(combinations(angle_combos, 2))
        points = []
        for combo in combos:
            r=10
            leftmost_point  = (
                    10*np.cos(combo[1])+agent_point[0],
                    10*np.sin(combo[1])+agent_point[1])

            rightmost_point = (
                    10*np.cos(combo[0])+agent_point[0],
                    10*np.sin(combo[0])+agent_point[1])
            points.append((leftmost_point, rightmost_point))
        angle_map = multiple_collision_check(agent_point, points, 45, 5)

        # print(n_l, n_r)
        # print(angle_map[n_l:n_r+1])
        print(angle_map)
        self.assertTrue(np.all(angle_map[0:2]) and np.all(angle_map[-2::]))
        self.assertFalse(np.any(angle_map[2:-2]))
    

    def test_correction(self):
        max_theta = 45
        step = 5
        r=10
        command = Command(5, np.pi/6)
        l_theta = np.deg2rad(35)
        r_theta = np.deg2rad(25)
        leftmost_point = (r*np.cos(l_theta)+10,r*np.sin(l_theta)+20)
        rightmost_point = (r*np.cos(r_theta)+10,r*np.sin(r_theta)+20)
        state = SimState(10, 20, 0, 0)
        safe_command = correct(state,
                               command,
                               leftmost_point,
                               rightmost_point,
                               None,
                               max_theta,
                               step)
        # print("Command: ")
        # print(np.rad2deg(safe_command[1]))
        self.assertTrue(np.abs(safe_command[1]-np.deg2rad(20))<1e-10)


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(OATests)
    unittest.TextTestRunner(verbosity=4).run(suite)
