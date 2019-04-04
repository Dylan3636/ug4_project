#include "ros_swarm_tools.h"
#include <ros/xmlrpc_manager.h>
void extract_from_world_msg(const swarm_msgs::worldStateConstPtr &world_state,
                            std::map<int, agent::AgentState> &usv_state_map,
                            std::map<int, agent::AgentState> &intruder_state_map,
                            agent::AgentState &asset_state){
    // auto ws = world_state->worldState;
    for ( const swarm_msgs::agentState &agent_state : world_state->worldState){
        if (agent_state.agent_type == swarm_msgs::agentType::USV){
            int sim_id = agent_state.sim_id;
            double x = agent_state.x;
            double y = agent_state.y;
            double speed = agent_state.speed;
            double heading = agent_state.heading;
            double radius = agent_state.radius;

            ROS_DEBUG("Found USV [%d]: \nx: [%f] \ny: [%f] \nspeed: [%f] \nheading: [%f], \nradius: [%f]",
                    sim_id,
                    x,
                    y,
                    speed,
                    heading*180/swarm_tools::PI,
                    radius
                    );

             usv_state_map[sim_id] = agent::AgentState(x, y, speed, heading, radius, sim_id);
        }
        if (agent_state.agent_type == swarm_msgs::agentType::INTRUDER){
            int sim_id = agent_state.sim_id;
            double x = agent_state.x;
            double y = agent_state.y;
            double speed = agent_state.speed;
            double heading = agent_state.heading;
            double radius = agent_state.radius;

            ROS_DEBUG("Found Intruder [%d]: \nx: [%f] \ny: [%f] \nspeed: [%f] \nheading: [%f], \nradius: [%f]",
                    sim_id,
                    x,
                    y,
                    speed,
                    heading*180/swarm_tools::PI,
                    radius
                    );

            intruder_state_map[sim_id] = agent::AgentState(x, y, speed, heading, radius, sim_id);
        }
        if (agent_state.agent_type == swarm_msgs::agentType::TANKER){
            int sim_id = agent_state.sim_id;
            double x = agent_state.x;
            double y = agent_state.y;
            double speed = agent_state.speed;
            double heading = agent_state.heading;
            double radius = agent_state.radius;

            ROS_DEBUG("Found Tanker [%d]: \nx: [%f] \ny: [%f] \nspeed: [%f] \nheading: [%f], \nradius: [%f]",
                    sim_id,
                    x,
                    y,
                    speed,
                    heading*180/swarm_tools::PI,
                    radius
                    );
            asset_state = agent::AgentState(x, y, speed, heading, radius, sim_id);
        }
    }
}

std::map<int, agent::ObservedIntruderAgent> extract_from_intruder_msgs(const std::vector<swarm_msgs::intruderAgent> &intruder_msgs){
    std::map<int, agent::ObservedIntruderAgent> intruder_map;
    for (auto const &intruder_msg : intruder_msgs){
        intruder_map[intruder_msg.state.sim_id]=extract_from_intruder_msg(intruder_msg);
    }
    return intruder_map;
}

agent::ObservedIntruderAgent extract_from_intruder_msg(const swarm_msgs::intruderAgent &intruder_msg){
    return agent::ObservedIntruderAgent{extract_from_state_msg(intruder_msg.state),
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

agent::AgentAssignment extract_from_assignment_msg(const swarm_msgs::agentAssignment &assignment_msg){

    agent::AgentAssignment assignment;
    for(const auto &task_msg : assignment_msg.tasks){
        assignment.push_back(extract_from_task_msg(task_msg));
    }
    return assignment;
}

agent::AgentTask extract_from_task_msg(const swarm_msgs::agentTask &task_msg){
    agent::AgentTask agent_task;
    agent_task.task_idx = task_msg.task_idx;
    switch(task_msg.task_type){
        case swarm_msgs::taskType::OBSERVE:
            agent_task.task_type=agent::TaskType::Observe;
            break;
        case swarm_msgs::taskType::GUARD:
            agent_task.task_type=agent::TaskType::Guard;
            break;
        case swarm_msgs::taskType::DELAY:
            agent_task.task_type=agent::TaskType::Delay;
            break;
    }
    return agent_task;
}

agent::AgentCommand extract_from_command_msg(const swarm_msgs::agentCommand &command_msg){
    return agent::AgentCommand(command_msg.delta_speed, command_msg.delta_heading);
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
    for(const auto &assignment_pair : swarm_assignment){
        swarm_assignment_msg.usvAssignments.push_back(
            convert_to_agent_assignment_msg(assignment_pair.first,
                                            assignment_pair.second));
    }
    return swarm_assignment_msg;
}

swarm_msgs::agentAssignment convert_to_agent_assignment_msg(int sim_id, const agent::AgentAssignment &agent_assignment){
    swarm_msgs::agentAssignment assignment_msg;
    for(const auto &task : agent_assignment){
        assignment_msg.tasks.push_back(convert_to_agent_task_msg(task));
    }
    assignment_msg.sim_id = sim_id;
    return assignment_msg;
}

swarm_msgs::agentTask convert_to_agent_task_msg(const agent::AgentTask &task){
    swarm_msgs::agentTask task_msg;
    task_msg.task_idx = task.task_idx;
    // ROS_INFO("CONVERTING TASK: %s TO MSG", task.to_string());
    switch(task.task_type){
        case agent::TaskType::Observe:
            task_msg.task_type=swarm_msgs::taskType::OBSERVE;
            break;
        case agent::TaskType::Guard:
            task_msg.task_type=swarm_msgs::taskType::GUARD;
            break;
        case agent::TaskType::Delay:
            task_msg.task_type=swarm_msgs::taskType::DELAY;
            break;
    }
    task_msg.task_idx = task.task_idx;
    return task_msg;
}
swarm_msgs::agentState convert_to_agent_state_msg(const agent::AgentState &state){
    swarm_msgs::agentState state_msg;
    state_msg.sim_id=state.sim_id;
    state_msg.x=state.x;
    state_msg.y=state.y;
    state_msg.heading=state.heading;
    state_msg.speed=state.speed;
    state_msg.radius=state.radius;
    // TODO state_msg expects agent type
    return state_msg;
}

bool get_complex_parameters(const RosContainerPtr &ros_container_ptr,
                            const std::string &head_str,
                            XmlRpc::XmlRpcValue &items){
    bool failed = !ros_container_ptr->nh.getParam(head_str, items);
    if(failed){
        ROS_ERROR("COULD NOT LOAD PARAMETERS FROM %s", head_str.c_str());
    }else{
        ROS_ASSERT(items.getType()==XmlRpc::XmlRpcValue::TypeArray);
        ROS_INFO("LOADED PARAMETERS FROM %s", head_str.c_str());
        ROS_INFO("HAS %d ITEMS", items.size());
    }
    return !failed;
}

void get_obstacle_states(
        const swarm_msgs::worldStateConstPtr &world_state,
        std::vector<agent::AgentState> &obstacle_states
        ){
    obstacle_states.clear();
    for(const auto agent_state : world_state->worldState){
        if(agent_state.agent_type==swarm_msgs::agentType::TANKER) continue;
        obstacle_states.emplace_back(agent_state.x,
                                      agent_state.y,
                                      agent_state.heading,
                                      agent_state.speed,
                                      agent_state.radius,
                                      agent_state.sim_id);
    }
}
template<typename T> void get_intruder_previous_states_msg(const T &intruder,
        swarm_msgs::intruderPreviousStates &msg){
    for(const auto &state : intruder.previous_states){
        msg.previous_states.push_back(convert_to_agent_state_msg(state));
    }
}


template<typename T> void get_batch_intruder_previous_states_msg(const std::vector<T> &intruders,
        swarm_msgs::batchIntruderPreviousStates &prev_states_msg){
    prev_states_msg.batch_previous_states.clear();
    for(const auto &intruder: intruders){
        auto msg = swarm_msgs::intruderPreviousStates();
        get_intruder_previous_states_msg(intruder, msg);
        prev_states_msg.batch_previous_states.push_back(msg);
    }
}

template void get_batch_intruder_previous_states_msg(const std::vector<agent::ObservedIntruderAgent> &intruders,
        swarm_msgs::batchIntruderPreviousStates &prev_states_msg);
template void get_batch_intruder_previous_states_msg(const std::vector<agent::IntruderAgent> &intruders,
                                                     swarm_msgs::batchIntruderPreviousStates &prev_states_msg);

bool get_intruder_threat_classification(
        const RosContainerPtr &ros_container_ptr,
        const std::string &head_str,
        int intruder_id){


    XmlRpc::XmlRpcValue items;
    bool failed = !get_complex_parameters(ros_container_ptr, head_str, items);
    if (failed) return !failed;

    XmlRpc::XmlRpcValue item;
    for(int i=0; i<items.size(); i++) {

        item = items[i];
        ROS_ASSERT(item.hasMember("sim_id"));
        auto sim_id_leaf = item["sim_id"];
        ROS_ASSERT(sim_id_leaf.getType() == XmlRpc::XmlRpcValue::TypeInt);
        int sim_id = static_cast<int>(sim_id_leaf);
        ROS_INFO("LOADED SIM ID %d", sim_id);

        // Skip if not correct intruder
        if (sim_id != intruder_id) {
            continue;
        } else {
            ROS_ASSERT(item.hasMember("is_threat"));
            auto is_threat_leaf = item["is_threat"];
            ROS_ASSERT(is_threat_leaf.getType() == XmlRpc::XmlRpcValue::TypeBoolean);
            bool threat = static_cast<bool>(is_threat_leaf);
            ROS_INFO("LOADED IS THREAT %d", threat);
            return threat;
        }
    }
}

bool get_intruder_motion_goals(
        const RosContainerPtr &ros_container_ptr,
        const std::string &head_str,
        int intruder_id,
        bool &threat,
        std::vector<agent::MotionGoal> &motion_goals){

    XmlRpc::XmlRpcValue items;
    bool failed = !get_complex_parameters(ros_container_ptr, head_str, items);
    if (failed) return !failed;
    motion_goals.clear();

    XmlRpc::XmlRpcValue item;
    for(int i=0; i<items.size(); i++){

        item = items[i];
        ROS_ASSERT(item.hasMember("sim_id"));
        auto sim_id_leaf = item["sim_id"];
        ROS_ASSERT(sim_id_leaf.getType()==XmlRpc::XmlRpcValue::TypeInt);
        int sim_id = static_cast<int>(sim_id_leaf);
        ROS_INFO("LOADED SIM ID %d", sim_id);

        // Skip if not correct intruder
        if(sim_id != intruder_id){
            continue;
        }else{
            ROS_ASSERT(item.hasMember("is_threat"));
            auto is_threat_leaf = item["is_threat"];
            ROS_ASSERT(is_threat_leaf.getType()==XmlRpc::XmlRpcValue::TypeBoolean);
            threat = static_cast<bool>(is_threat_leaf);
            ROS_INFO("LOADED IS THREAT %d", threat);
            if(threat) return true;
            return false;
            ROS_ASSERT(item.hasMember("motion_goals"));
            auto motion_goals_branch = item["motion_goals"];
            ROS_ASSERT(motion_goals_branch.getType()==XmlRpc::XmlRpcValue::TypeArray);
            for(int_fast32_t j=0; j<motion_goals_branch.size(); j++){
                agent::MotionGoal mg;
                auto mg_item = motion_goals_branch[j];
                ROS_ASSERT(mg_item.getType()==XmlRpc::XmlRpcValue::TypeStruct);

                ROS_ASSERT(mg_item.hasMember("x"));
                auto mg_x_leaf = mg_item["x"];
                ROS_ASSERT(mg_x_leaf.getType()==XmlRpc::XmlRpcValue::TypeDouble);
                mg.x = static_cast<double>(mg_x_leaf);

                ROS_ASSERT(mg_item.hasMember("y"));
                auto mg_y_leaf = mg_item["y"];
                ROS_ASSERT(mg_y_leaf.getType()==XmlRpc::XmlRpcValue::TypeDouble);
                mg.y = static_cast<double>(mg_y_leaf);
                motion_goals.push_back(mg);
            }
            return true;
        }

   }
}
bool get_agent_parameters(RosContainerPtr ros_container_ptr,
                          std::string head_str,
                          std::map<int, agent::AgentConstraints> &constraints_map,
                          std::map<int, agent::CollisionAvoidanceParameters> &radar_params_map){

    XmlRpc::XmlRpcValue items;
    bool failed = !get_complex_parameters(ros_container_ptr, head_str, items);
    if (failed) return !failed;

    for(int_fast32_t i=0; i<items.size(); i++){
        auto item = items[i];
        ROS_ASSERT(item.hasMember("sim_id"));
        int sim_id = static_cast<int>(item["sim_id"]);


        // LOADING AGENT'S CONSTRAINTS
        agent::AgentConstraints constraints;
        ROS_ASSERT(item.hasMember("constraints"));
        auto constraints_branch = item["constraints"];

        ROS_ASSERT(constraints_branch.hasMember("max_speed"));
        auto max_speed_leaf = constraints_branch["max_speed"];
        ROS_ASSERT(max_speed_leaf.getType()==XmlRpc::XmlRpcValue::TypeDouble);
        constraints.max_speed = static_cast<double >(max_speed_leaf);
        ROS_INFO("LOADED PARAMETER max_speed %f", constraints.max_speed);

        ROS_ASSERT(constraints_branch.hasMember("max_delta_speed"));
        auto max_delta_speed_leaf = constraints_branch["max_delta_speed"];
        ROS_ASSERT(max_delta_speed_leaf.getType()==XmlRpc::XmlRpcValue::TypeDouble);
        constraints.max_delta_speed = static_cast<double >(max_delta_speed_leaf);
        ROS_INFO("LOADED PARAMETER max_delta_speed %f", constraints.max_delta_speed);

        ROS_ASSERT(constraints_branch.hasMember("max_delta_heading"));
        auto max_delta_heading_leaf = constraints_branch["max_delta_heading"];
        ROS_ASSERT(max_delta_heading_leaf.getType()==XmlRpc::XmlRpcValue::TypeDouble);
        constraints.max_delta_heading = swarm_tools::deg2rad(static_cast<double >(max_delta_heading_leaf));
        ROS_INFO("LOADED PARAMETER max_delta_heading %f", swarm_tools::rad2deg(constraints.max_delta_heading));
        constraints_map[sim_id] = constraints;

        // LOADING AGENT'S RADAR PARAMETERS
        agent::CollisionAvoidanceParameters radar_params;
        ROS_ASSERT(item.hasMember("radar_parameters"));
        auto radar_branch = item["radar_parameters"];

        ROS_ASSERT(radar_branch.hasMember("max_distance"));
        auto max_distance_leaf = radar_branch["max_distance"];
        ROS_ASSERT(max_distance_leaf.getType()==XmlRpc::XmlRpcValue::TypeDouble);
        radar_params.max_radar_distance = static_cast<double >(max_distance_leaf);
        ROS_INFO("LOADED PARAMETER max_distance %f", radar_params.max_radar_distance);

        ROS_ASSERT(radar_branch.hasMember("max_angle"));
        auto max_angle_leaf = radar_branch["max_angle"];
        ROS_ASSERT(max_angle_leaf.getType()==XmlRpc::XmlRpcValue::TypeDouble);
        radar_params.max_radar_angle_rad = swarm_tools::deg2rad(static_cast<double >(max_angle_leaf));
        ROS_INFO("LOADED PARAMETER max_angle %f", swarm_tools::rad2deg(radar_params.max_radar_angle_rad));

        ROS_ASSERT(radar_branch.hasMember("aggression"));
        auto aggression_leaf= radar_branch["aggression"];
        ROS_ASSERT(aggression_leaf.getType()==XmlRpc::XmlRpcValue::TypeDouble);
        radar_params.aggression = static_cast<double >(aggression_leaf);
        ROS_INFO("LOADED PARAMETER aggression %f", radar_params.aggression);
        radar_params_map[sim_id] = radar_params;
    }
    return true;
}
