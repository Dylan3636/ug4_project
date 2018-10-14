import numpy as np
import unittest
from sim import *
from sim_objects import *
from helper_tools import close_enough

class SimTests(unittest.TestCase):

    def test_sim_object_init(self):
        obj = SimObject(0)
        
        assert obj.sim_id == 0
        assert obj.x == 0
        assert obj.y == 0
        assert obj.speed == 0
        assert obj.heading == 0


        obj.command = [1, np.pi/6]
        assert(obj.command == [1, np.pi/6])

    def test_sim_object_update(self):
        obj = SimObject(0)

        obj.x = 1
        obj.y = 1
        obj.speed = 1
        obj.heading = np.pi/6

        obj.command = [1, np.pi/6]
        assert(obj.command == [1, np.pi/6])
        
        obj.update_state(1)
        assert close_enough(obj.x, 1+np.sqrt(3)/2),\
                            "{} != {}".format(obj.x,
                                              1+np.sqrt(3)/2)
        assert close_enough(obj.y, 1.5),\
                "{} != {}".format(obj.y,
                                  1.5)
        assert obj.speed == 2,\
                "{} != {}".format(obj.speed,
                        2)
        assert obj.heading == np.pi/3,\
                "{} != {}".format(obj.heading, np.pi/3)
        assert obj.command == [0, 0],\
                "command didn't return to zeros"
        

if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(SimTests)
    unittest.TextTestRunner(verbosity=4).run(suite)
