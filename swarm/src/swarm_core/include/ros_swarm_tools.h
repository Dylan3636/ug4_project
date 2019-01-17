#include <map>
#include <ros/ros.h>
#include "usv_swarm.h"
#include "swarm_msgs/agentType.h"
#include "swarm_msgs/agentState.h"
#include "swarm_msgs/agentAssignment.h"
#include "swarm_msgs/agentCommand.h"
#include "swarm_msgs/worldState.h"
#include "swarm_msgs/usvAgent.h"
#include "swarm_msgs/intruderAgent.h"
#include "swarm_msgs/swarmAssignment.h"

#ifndef SWARM_CORE_ROS_SWARM_TOOLS_H
#define SWARM_CORE_ROS_SWARM_TOOLS_H

struct RosInit{
    RosInit(int argc, char** argv, const char *node_name){
        ros::init(argc, argv, node_name);
    }
};

struct RosContainer{

    RosInit ros_initializer;
    ros::NodeHandle nh;

    RosContainer(int argc, char** argv, const char *node_name) : ros_initializer(argc, argv, node_name){}
};

typedef boost::shared_ptr<RosContainer> RosContainerPtr;

void extract_from_world_msg(const swarm_msgs::worldStateConstPtr &world_state,
                            std::map<int, agent::AgentState> &usv_state_map,
                            std::map<int, agent::AgentState> &intruder_state_map,
                            agent::AgentState &asset_state);

agent::AgentState extract_from_state_msg(
    const swarm_msgs::agentState &state_msg);

agent::AgentConstraints extract_from_constraints_msg(
    const swarm_msgs::agentConstraints &constraints_msg);

agent::CollisionAvoidanceParameters extract_from_ca_params_msg(
    const swarm_msgs::agentParam &ca_params_msg);

agent::AgentAssignment extract_from_assignment_msg(
    const swarm_msgs::agentAssignment assignment_msg);

agent::AgentCommand extract_from_command_msg(
    const swarm_msgs::agentCommand &command_msg);

agent::USVAgent extract_from_usv_msg(
    const swarm_msgs::usvAgent &usv_msg);

std::map<int, agent::USVAgent> extract_from_usv_msgs(
    const std::vector<swarm_msgs::usvAgent> &usv_msgs);

agent::IntruderAgent extract_from_intruder_msg(
    const swarm_msgs::intruderAgent &intruder_msg);

std::map<int, agent::IntruderAgent> extract_from_intruder_msgs(
    const std::vector<swarm_msgs::intruderAgent> &intruder_msgs);

agent::SwarmAssignment extract_from_swarm_assignment_msg(
    const swarm_msgs::swarmAssignment &swarm_assignment_msg);

swarm_msgs::swarmAssignment convert_to_swarm_assignment_msg(
    const agent::SwarmAssignment &swarm_assignment);
swarm_msgs::agentAssignment convert_to_agent_assignment_msg(
    int sim_id, const agent::AgentAssignment &agent_assignment);
#endif