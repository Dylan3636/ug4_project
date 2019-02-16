from time import sleep
from perception import perceive
from planning import avoid
from sim_objects import *
from time import time
from plot import LivePlot

SimLine = namedtuple("SimulationLine", ["sim_id", "start_id", "end_id"])


class Simulation:

    def __init__(self, sim_objects, timeout=0.1, delta_time_secs=None, use_gui=True, threshold=None):
        self.sim_objects = dict([(obj.sim_id, obj) for obj in sim_objects])
        self.OK = True
        self.use_gui = use_gui
        self.anim = LivePlot()
        self.timeout = timeout
        self.delta_t = timeout if delta_time_secs is None else delta_time_secs
        self.start_time = time()
        self.end_time = None
        self.threshold = threshold

    def update_simulation_state(self):
        # Overridden in simulation node
        readings = perceive(self.sim_objects)
        for sim_id, obj in self.sim_objects.items():
            if obj.object_type != "STATIC":
                obj.command = avoid(obj, readings)
            obj.update_state(self.timeout)

    def command_by_object_id(self, sim_id, command):
        self.sim_objects[sim_id].command = command

    def update_task_lines(self, assignments):
        line_updates = {}
        for tasks in assignments:
            for (usv_id, task_type, task_id) in tasks:
                if task_type == 1 or task_id == -1:
                    continue
                line_id = get_line_id(usv_id, task_id)
                line_updates[line_id] = SimLine(line_id, usv_id, task_id)

        self.anim.update_lines(line_updates)
 
    def begin(self):
        while self.OK:

            self.update_simulation_state()

            start = time()
            if self.use_gui:
                # Visualisation
                self.anim.update_world_state(self.sim_objects)
                self.anim.cull_objects()
                self.anim.update_threat_color()

            # Termination check
            terminate = self.termination_check()
            if terminate:
                self.kill()
            end = time()
            # Timeout
            sleep(max(self.timeout-(end-start), 0))

    def termination_check(self):
        if self.threshold is None:
            return False
        terminate = False
        tanker = list(filter(lambda x: type(x) == Tanker, self.sim_objects))[0]
        intruders = filter(lambda x: type(x) == Intruder, self.sim_objects)

        for obj in intruders:
            dist = euclidean_distance(obj.position, tanker.position)
            terminate |= dist < self.threshold
        return terminate

    def kill(self):
        assert not self.OK, "SIMULATION IS ALREADY DEAD"
        self.OK = False
        self.end_time = time()


def get_line_id(sim_id, task_id):
    return sim_id*500 + task_id


if __name__ == '__main__':
    usv_1 = BasicUSV(0, [0, 75, 30, 0], radius_buffer=40)
    usv_2 = BasicUSV(1, [300, 75, 30, np.pi], radius_buffer=40)
    static_1 = StaticObject(100, [200, 100, 0, 0], radius_buffer=30)
    static_2 = StaticObject(101, [250, 75, 0, 0], radius_buffer=30)

    sim = Simulation([usv_1, usv_2, static_1, static_2],
                     0.1)

    sim.begin()
    sim.anim.canvas.mainloop()

