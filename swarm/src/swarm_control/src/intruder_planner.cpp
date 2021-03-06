#include <map>
#include <vector>
#include "agent.h"
#include "collision_avoidance.h"
#include "motion_goal_control.h"
#include "ros_swarm_tools.h"
#include "swarm_msgs/resetSystem.h"
#include "swarm_msgs/initializeSystem.h"
#include "swarm_msgs/agentCommand.h"
//#include "swarm_control/CollisionAvoidance.h"
#include "swarm_threat_detection/batchIntruderCommands.h"

int count=0;
bool initialized=false;
bool use_model=false;
ros::Publisher command_pub;
ros::ServiceClient client;
ros::ServiceClient model_client;
RosContainerPtr ros_container_ptr;
std::map<int, agent::IntruderAgent> intruder_map;
std::map<int, agent::CollisionAvoidanceParameters> intruder_radar_params_map;
std::map<int, agent::AgentConstraints> intruder_constraints_map;
std::string intruder_head_str = "/swarm_simulation/intruder_params";

void initialize_callback(const swarm_msgs::initializeSystem &msg){
    if(initialized) return;
    initialized = true;
    ROS_INFO("Initialized Intruders");
}

void reset_callback(const swarm_msgs::resetSystem &msg){
    if (!initialized) return;
    intruder_map.clear();
    intruder_constraints_map.clear();
    intruder_radar_params_map.clear();
    initialized = false;
    ROS_INFO("Reset Intruders");
}

void callback(const swarm_msgs::worldState::ConstPtr& world_state){
    if(!initialized) return;
    auto ws = world_state->worldState;

    agent::AgentState tanker;
    for ( const swarm_msgs::agentState &agent_state : ws){
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

            if (intruder_map.find(sim_id)!=intruder_map.end()){
                ROS_INFO("Updating Intruder %d", sim_id);
                intruder_map[sim_id].update_state(x, y, speed, heading);
            }
            else{
                ROS_INFO("Adding Intruder %d", sim_id);

                // Get Intruder Parameters
                agent::AgentState intruder_state {x, y, speed, heading, radius, sim_id};
                auto intruder = agent::IntruderAgent(
                        true,
                        intruder_state,
                        intruder_constraints_map[sim_id],
                        intruder_radar_params_map[sim_id]);

                // Get Intruder Motion Goals
//                auto mgs = intruder.get_motion_goals_ref();
                bool threat=get_intruder_threat_classification(
                        ros_container_ptr,
                        intruder_head_str,
                        sim_id);
                intruder.set_threat_classification(threat);
                intruder_map[sim_id] = intruder;
            }

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
            tanker = {x, y, speed, heading, radius, sim_id}; 
        }
    }
    std::vector<agent::AgentState> obstacle_states;
    get_obstacle_states(world_state, obstacle_states);
    auto obstacle_states_w_asset = obstacle_states;
    obstacle_states_w_asset.push_back(agent::AgentState(0,0,0,0,150,1));
    std::map<int, agent::AgentCommand> intruder_commands_map;

    std::vector<agent::IntruderAgent> intruders;

    for (auto &intruder_pair : intruder_map){
        intruders.push_back(intruder_pair.second);
    }

    if (use_model){
        swarm_control::get_batch_intruder_commands_from_model(intruders, intruder_commands_map, model_client);
    }
    for (auto &intruder_pair : intruder_map){

        int sim_id = intruder_pair.first;
        auto &intruder = intruder_pair.second;

        agent::AgentCommand command;
        agent::MotionGoal motion_goal;
        // bool end = !intruder.get_motion_goal(&motion_goal);
        ROS_INFO("Intruder %d, %d", intruder_pair.first, intruder.is_threat());
        if(!intruder.is_threat()){
            motion_goal.x=-4000;
            motion_goal.y=1000;
        }
        //       ROS_ASSERT(!end);

        if(intruder.is_threat()){
            swarm_control::get_intruder_command_from_motion_goal(intruder,
                                                                 motion_goal,
                                                                 command);
            ROS_INFO("Motion goal ([%f], [%f])",
                     motion_goal.x, motion_goal.y);
            ROS_INFO("Position ([%f], [%f])",
                     intruder.get_x(), intruder.get_y());
        }
        else{
            command.delta_heading=swarm_control::get_smallest_delta_heading(
                    intruder.heading_goal,
                    intruder.get_heading(),
                    intruder.get_max_delta_heading());
        }

        ROS_DEBUG("Correcting command for intruder [%d]", sim_id);
        ROS_DEBUG("Recommended command ([%f], [%f])",
                command.delta_speed,
                command.delta_heading*180/swarm_tools::PI);

        bool evade = intruder.update_evade();
        double aggression = intruder.get_aggression();
        int result;
        if(intruder.is_threat()){
            result = collision_avoidance::correct_command(intruder,
                                                              obstacle_states,
                                                              command);
        }else{
            result = collision_avoidance::correct_command(intruder,
                                                              obstacle_states_w_asset,
                                                              command);
        }
        if (result!=0){
            intruder.update_previously_blocked_time();
        }
        ROS_DEBUG("Collision Avoidance Result %d.", result);
        ROS_DEBUG("Corrected command ([%f], [%f])",
                command.delta_speed,
                command.delta_heading*180/swarm_tools::PI);
        auto command_msg = swarm_msgs::agentCommand();
        command_msg.delta_speed = command.delta_speed;
        command_msg.delta_heading = command.delta_heading;
        command_msg.sim_id = sim_id;
        command_pub.publish(command_msg);
    }
}

int main(int argc, char **argv){
    ros_container_ptr.reset(new RosContainer(argc, argv, "intruder_control"));
    command_pub = ros_container_ptr->nh.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    ROS_INFO("Starting Intruder Control Node");
    ros::Subscriber sub = ros_container_ptr->nh.subscribe("Perception", 1000, callback);
    ros::Subscriber reset_sub = ros_container_ptr->nh.subscribe("SystemReset", 100, reset_callback);
    ros::Subscriber init_sub = ros_container_ptr->nh.subscribe("SystemStart", 100, initialize_callback);
//    client = ros_container_ptr->nh.serviceClient<swarm_control::CollisionAvoidance>("collision_avoidance");
    model_client = ros_container_ptr->nh.serviceClient<swarm_threat_detection::batchIntruderCommands>("swarm_intruder_model_service");

    // Spin until initialized
    ros::Rate loop_rate(20);
    while(ros::ok()) {
        ros::spinOnce();
        if(!initialized){
            continue;
        }else{
            if(intruder_constraints_map.empty()){
                // Load Parameters
                get_agent_parameters(ros_container_ptr,
                                     intruder_head_str,
                                     intruder_constraints_map,
                                     intruder_radar_params_map);
            }
        }
        loop_rate.sleep();
    }
    return 0;
}