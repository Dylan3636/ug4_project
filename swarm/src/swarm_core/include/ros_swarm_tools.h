#include <map>
#include <ros/ros.h>
#include "agent.h"
#include "swarm_msgs/agentType.h"
#include "swarm_msgs/agentState.h"
#include "swarm_msgs/worldState.h"

void extract_from_world_msg(const swarm_msgs::worldStateConstPtr &world_state,
                            std::map<int, agent::AgentState> &usv_state_map,
                            std::map<int, agent::AgentState> &intruder_state_map,
                            agent::AgentState &asset_state);