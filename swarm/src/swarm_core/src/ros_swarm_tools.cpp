#include "ros_swarm_tools.h"

void extract_from_world_msg(const swarm_msgs::worldStateConstPtr &world_state,
                            std::map<int, agent::AgentState> &usv_state_map,
                            std::map<int, agent::AgentState> &intruder_state_map,
                            agent::AgentState &asset_state){
    // auto ws = world_state->worldState;
    for ( swarm_msgs::agentState agent_state : world_state->worldState){
        if (agent_state.agent_type == swarm_msgs::agentType::USV){
            int sim_id = agent_state.sim_id;
            double x = agent_state.x;
            double y = agent_state.y;
            double speed = agent_state.speed;
            double heading = agent_state.heading;
            double radius = agent_state.radius;

            ROS_INFO("Found USV [%d]: \nx: [%f] \ny: [%f] \nspeed: [%f] \nheading: [%f], \nradius: [%f]",
                    sim_id,
                    x,
                    y,
                    speed,
                    heading*180/swarm_tools::PI,
                    radius
                    );

            agent::AgentState usv_state = {x, y, speed, heading, radius, sim_id}; 
            usv_state_map[sim_id] = usv_state;
        }
        if (agent_state.agent_type == swarm_msgs::agentType::INTRUDER){
            int sim_id = agent_state.sim_id;
            double x = agent_state.x;
            double y = agent_state.y;
            double speed = agent_state.speed;
            double heading = agent_state.heading;
            double radius = agent_state.radius;

            ROS_INFO("Found Intruder [%d]: \nx: [%f] \ny: [%f] \nspeed: [%f] \nheading: [%f], \nradius: [%f]",
                    sim_id,
                    x,
                    y,
                    speed,
                    heading*180/swarm_tools::PI,
                    radius
                    );

            agent::AgentState intruder_state = {x, y, speed, heading, radius, sim_id};
            intruder_state_map[sim_id] = intruder_state;
        }
        if (agent_state.agent_type == swarm_msgs::agentType::TANKER){
            int sim_id = agent_state.sim_id;
            double x = agent_state.x;
            double y = agent_state.y;
            double speed = agent_state.speed;
            double heading = agent_state.heading;
            double radius = agent_state.radius;

            ROS_INFO("Found Tanker [%d]: \nx: [%f] \ny: [%f] \nspeed: [%f] \nheading: [%f], \nradius: [%f]",
                    sim_id,
                    x,
                    y,
                    speed,
                    heading*180/swarm_tools::PI,
                    radius
                    );
            asset_state = {x, y, speed, heading, radius, sim_id}; 
        }
    }
}

std::map<int, agent::IntruderAgent> extract_from_intruder_msgs(const std::vector<swarm_msgs::intruderAgent> &intruder_msgs){
    std::map<int, agent::IntruderAgent> intruder_map;
    for (auto const &intruder_msg : intruder_msgs){
        intruder_map[intruder_msg.state.sim_id]=extract_from_intruder_msg(intruder_msg);
    }
    return intruder_map;
}

agent::IntruderAgent extract_from_intruder_msg(const swarm_msgs::intruderAgent &intruder_msg){
    return agent::IntruderAgent{extract_from_state_msg(intruder_msg.state),
                                extract_from_constraints_msg(intruder_msg.constraints),
                                extract_from_ca_params_msg(intruder_msg.ca_params)};
}

std::map<int, agent::USVAgent> extract_from_usv_msgs(const std::vector<swarm_msgs::usvAgent> &usv_msgs){
    std::map<int, agent::USVAgent> usv_map;
    for (auto const &usv_msg : usv_msgs){
        usv_map[usv_msg.state.sim_id]=extract_from_usv_msg(usv_msg);
    }
    return usv_map;
}

agent::USVAgent extract_from_usv_msg(const swarm_msgs::usvAgent &usv_msg){
    return agent::USVAgent(extract_from_state_msg(usv_msg.state),
                           extract_from_constraints_msg(usv_msg.constraints),
                           extract_from_ca_params_msg(usv_msg.ca_params),
                           extract_from_assignment_msg(usv_msg.assignment));
}


agent::AgentState extract_from_state_msg(const swarm_msgs::agentState &state_msg){
    return agent::AgentState{state_msg.x,
                             state_msg.y,
                             state_msg.speed,
                             state_msg.heading,
                             state_msg.radius,
                             state_msg.sim_id};
}

agent::AgentConstraints extract_from_constraints_msg(const swarm_msgs::agentConstraints &constraints_msg){
    return agent::AgentConstraints{constraints_msg.max_speed,
                                   constraints_msg.max_delta_speed,
                                   constraints_msg.max_delta_heading};
}

agent::CollisionAvoidanceParameters extract_from_ca_params_msg(const swarm_msgs::agentParam &ca_params_msg){
    return agent::CollisionAvoidanceParameters{ca_params_msg.max_distance,
                                               ca_params_msg.max_angle,
                                               ca_params_msg.aggression};
}

agent::AgentAssignment extract_from_assignment_msg(const swarm_msgs::agentAssignment assignment_msg){
    return agent::AgentAssignment{assignment_msg.delay_assignment, assignment_msg.guard_assignment};
}

agent::AgentCommand extract_from_command_msg(const swarm_msgs::agentCommand &command_msg){
    return agent::AgentCommand{command_msg.delta_speed, command_msg.delta_heading};
}

agent::SwarmAssignment extract_from_swarm_assignment_msg(const swarm_msgs::swarmAssignment &swarm_assignment_msg){
    agent::SwarmAssignment swarm_assignment;
    for (const auto &agent_assignment_msg : swarm_assignment_msg.usvAssignments){
        auto agent_assignment = extract_from_assignment_msg(agent_assignment_msg);
        swarm_assignment[agent_assignment_msg.sim_id] = agent_assignment;
    }
    return swarm_assignment;
}

swarm_msgs::swarmAssignment convert_to_swarm_assignment_msg(const agent::SwarmAssignment &swarm_assignment){
    swarm_msgs::swarmAssignment swarm_assignment_msg;
    swarm_assignment_msg.usvAssignments.clear();
    for(const auto &assignment_pair : swarm_assignment){
        swarm_assignment_msg.usvAssignments.push_back(
            convert_to_agent_assignment_msg(assignment_pair.first,
                                            assignment_pair.second));
    }
    return swarm_assignment_msg;
}

swarm_msgs::agentAssignment convert_to_agent_assignment_msg(int sim_id, const agent::AgentAssignment &agent_assignment){
    swarm_msgs::agentAssignment assignment_msg;
    assignment_msg.delay_assignment= agent_assignment.delay_assignment_idx;
    assignment_msg.guard_assignment= agent_assignment.guard_assignment_idx;
    assignment_msg.sim_id = sim_id;
    return assignment_msg;
}