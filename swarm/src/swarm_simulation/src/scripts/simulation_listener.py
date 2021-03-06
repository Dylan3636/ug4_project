import numpy as np
import rospy
from sim import Simulation
from sim_objects import *
from swarm_msgs.msg import (agentState,
                            agentCommand,
                            agentType,
                            simulationMarker,
                            swarmAssignment,
                            threatStatistics,
                            worldState,
                            initializeSystem,
                            taskType
                            )
from time import time
import os
from perception import perceive


class SimulationNode(Simulation):

    def __init__(self,
                 sim_objects,
                 sim_timeout=0.1,
                 delta_t=None,
                 anim=None,
                 use_gui=True,
                 threshold=None,
                 initialize=True,
                 noise=5, max_time=60, stats=None):

        super().__init__(sim_objects, anim=anim, timeout=sim_timeout, delta_time_secs=delta_t, use_gui=use_gui,
                         threshold=threshold, max_time=max_time)
        if initialize:
            rospy.init_node("Simulation", anonymous=True)
        self._active_ids = None
        self.publisher = rospy.Publisher('Perception', worldState, queue_size=100)
        self.start_pub = rospy.Publisher('SystemStart', initializeSystem, queue_size=100)
        self.listener = rospy.Subscriber("Commands", agentCommand, self.command_callback)
        self.marker_listener = rospy.Subscriber("Markers", simulationMarker, self.marker_callback)
        self.threat_listener = rospy.Subscriber("threatStatistics", threatStatistics, self.threat_stats_callback)
        self.task_allocation_listener = rospy.Subscriber("Task_Allocation",
                                                         swarmAssignment,
                                                         self.task_allocation_callback)
        self.noise = noise
        self.stats = stats

    def command_callback(self, msg):
        command = Command(msg.delta_speed,
                          msg.delta_heading)
        self.command_by_object_id(msg.sim_id, command)

    def marker_callback(self, msg):
        self.anim.update_marker(msg)

    def threat_stats_callback(self, msg):
        self.anim.threat_color_callback(msg)

    def task_allocation_callback(self, msg):
        new_task_assignments = [[(usv_assignment.sim_id,
                                  task.task_type,
                                  task.task_idx) for task in usv_assignment.tasks]
                                for usv_assignment in msg.usvAssignments]
        self.update_task_lines([*new_task_assignments])
        self.update_assignment_stats([*new_task_assignments])

    def update_assignment_stats(self, task_assignments):
        intruder_ids = self.stats.intruder_times.keys()
        visisted = dict(zip(intruder_ids, [False]*len(intruder_ids)))
        for task in task_assignments:
            for (usv_id, task_type, task_idx) in task:
                if task_type == taskType.DELAY:
                    if task_idx in self.stats.intruder_times:
                        if self.stats.intruder_times[task_idx][1] == -1:
                            self.stats.intruder_times[task_idx][1] = time()
                            self.stats.intruder_times[task_idx][0] = self.start_time
                            visisted[task_idx]=True
                    else:
                        self.stats.num_false_pos += 1
                elif task_type == taskType.OBSERVE:
                    if task_idx in self.stats.intruder_times:
                        if self.stats.intruder_times[task_idx][1] != -1:
                            self.stats.num_true_neg += 1
                            visisted[task_idx]=True

        for t_id, visit in visisted.items():
            if not visit:
                self.stats.num_true_neg += 1


    @property
    def active_usvs(self):
        if self._active_ids is not None:
            return self._active_ids
        return [usv_id for usv_id, obj in self.sim_objects.items() if type(obj) is BasicUSV]

    def update_simulation_state(self):
        ws = worldState().worldState
        for sim_id, obj in self.sim_objects.items():
            # if type(obj) == Intruder and obj.activate_time != -1 and not obj.active:
                # intruder_configs=rospy.get_param('/swarm_simulation/intruder_params')
                # rospy.logerr("Intruder Config Before {}".format(intruder_configs))
                # if time() > self.start_time + obj.activate_time and not obj.active:
                #     rospy.logerr("Starting intruder {} as threat".format(sim_id))
                #     intruder_configs=rospy.get_param('/swarm_simulation/intruder_params')
                #     for config in intruder_configs:
                #         if config['sim_id']==sim_id:
                #             config['threat'] = True
                #     os.system('rosparam delete /swarm_simulation/{}'.format('intruder_params'))
                #     rospy.set_param('/swarm_simulation/intruder_params', intruder_configs)
                #     obj.active = True
                #     self.stats.intruder_times[obj.sim_id] = [time(), -1]
            ws.append(state_to_msg(sim_id, perceive(obj.update_state(self.timeout), self.noise)))
        self.publisher.publish(ws)

    def begin(self):
        start_msg = initializeSystem()
        start_msg.usv_ids = self.active_usvs
        rospy.loginfo("Publishing Initialization msg %s", str(start_msg))
        rate = rospy.Rate(10)
        t = time()
        while time()-t < 2:
            self.start_pub.publish(start_msg)
            rate.sleep()
        super().begin()
        self.stats.start_time=self.start_time
        self.stats.end_time=self.end_time
        return self.stats

    def unregister(self):
        self.listener.unregister()
        self.threat_listener.unregister()
        self.marker_listener.unregister()
        self.task_allocation_listener.unregister()
        self.publisher.unregister()
        self.start_pub.unregister()
        self.anim.reset()


def state_to_msg(sim_id, sim_state):
    msg = agentState()
    msg.x = sim_state.x
    msg.y = sim_state.y
    msg.speed = sim_state.speed
    msg.heading = sim_state.heading
    msg.radius = sim_state.radius
    msg.sim_id = sim_id
    if sim_state.object_type == "USV":
        msg.agent_type = agentType().USV
    elif sim_state.object_type == "INTRUDER":
        msg.agent_type = agentType().INTRUDER
    elif sim_state.object_type == "TANKER":
        msg.agent_type = agentType().TANKER
    else:
        msg.agent_type = agentType().STATIC
    return msg


if __name__ == "__main__":
    usv_1 = BasicUSV(0, [-300,75,30,0], radius_buffer=40)
    usv_2 = BasicUSV(1, [300,75,30,np.pi], radius_buffer=40)
    static_1 = StaticObject(100, [0, 100, 0, 0], radius_buffer=30)
    static_2 = StaticObject(101, [0, 75, 0, 0], radius_buffer=30)
    sn = SimulationNode([usv_1, usv_2, static_1, static_2])
    sn.begin()

