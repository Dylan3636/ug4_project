import numpy as np
import rospy
from sim import Simulation
from sim_objects import *
from swarm_msgs.msg import (agentState,
                            agentCommand,
                            agentType,
                            simulationMarker,
                            swarmAssignment,
                            worldState)

class SimulationNode:

    def __init__(self, sim_objects, delta_t=0.1):
        rospy.init_node("Simulation", anonymous=True)
        self.sim = Simulation(sim_objects, timeout=delta_t)
        self.sim.update_simulation_state = self.tick
        self.publisher = rospy.Publisher('Perception', worldState, queue_size=10)
        self.listener = rospy.Subscriber("Commands", agentCommand, self.command_callback)
        self.marker_listener = rospy.Subscriber("Markers", simulationMarker, self.marker_callback)
        self.task_allocation_listener = rospy.Subscriber("Task_Allocation", swarmAssignment, self.task_allocation_callback)


    def command_callback(self, msg):
        command = Command(msg.delta_speed,
                          msg.delta_heading)
        self.sim.command_by_object_id(msg.sim_id,
                                      command)
    def marker_callback(self, msg):
        self.sim.anim.update_marker(msg)

    def task_allocation_callback(self, msg):
        new_task_assignments = [[(usv_assignment.sim_id, task.task_type, task.task_idx)
                 for task in usv_assignment.tasks] \
                    for usv_assignment in msg.usvAssignments]
        rospy.loginfo(str([*new_task_assignments]))
        self.sim.update_task_lines([*new_task_assignments])
    def tick(self):
        ws = worldState().worldState
        [ws.append(state_to_msg(sim_id, obj.update_state(self.sim.timeout))) for sim_id, obj in self.sim.sim_objects.items()]
        self.publisher.publish(ws)
    
    

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
    sn.sim.begin()

