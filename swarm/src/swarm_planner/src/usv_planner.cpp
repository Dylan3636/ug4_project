// #include <string>
// #include <ros/ros.h>
// #include <boost/format.hpp>
// #include <boost/shared_ptr>
#include <mutex>
#include "motion_goal_control.h"
#include "collision_avoidance.h"
#include "ros_swarm_tools.h"
#include "swarm_msgs/simulationMarker.h"
#include "swarm_msgs/threatStatistics.h"
#include "task_allocation.h"
#include "swarm_threat_detection/ThreatDetection.h"
#include "swarm_task_manager/modelPredictiveSimulation.h"

bool initialized = false;
int usv_id;
int intruder_id;
int NUM_USVS;

agent::USVSwarm swarm;

// // Declare ros container
RosContainerPtr ros_container_ptr;
ros::Publisher command_pub;
ros::Publisher marker_pub;
ros::Publisher threat_pub;
ros::ServiceClient client;

std::map<int, ros::ServiceClient> usv_sync_service_map;
std::map<int, bool> usv_communication_map;
std::map<int, bool> usv_sync_map;
std::map<int, agent::AgentState> usv_state_map;
std::map<int, agent::AgentConstraints> usv_constraints_map;
std::map<int, agent::CollisionAvoidanceParameters> usv_radar_params_map;
std::map<int, agent::AgentState> intruder_state_map;
std::map<int, agent::AgentConstraints> intruder_constraints_map;
std::map<int, agent::CollisionAvoidanceParameters> intruder_radar_params_map;
agent::AgentState asset_state;

std::mutex mtx;

void publish_markers(const agent::MotionGoal &motion_goal){
    // Motion Goal 
    ROS_INFO("Publishing Motion Goal ([%f], [%f])", motion_goal.x, motion_goal.y);
    swarm_msgs::simulationMarker marker;
    marker.x = motion_goal.x;
    marker.y = motion_goal.y;
    marker.sim_id = 200*usv_id;
    marker.colour = "RED";
    marker_pub.publish(marker);
}

void publish_threat_statistics(int intruder_id, double probability, bool classification){
    swarm_msgs::threatStatistics stats;
    stats.intruder_id=intruder_id;
    stats.threat_probability = probability;
    stats.threat_classification =classification;
    threat_pub.publish(stats);
}

void callback(const swarm_msgs::worldState::ConstPtr& world_state_ptr){

    ROS_INFO("USV CALLBACK");
    usv_state_map.clear();
    intruder_state_map.clear();

    extract_from_world_msg(world_state_ptr,
                           usv_state_map,
                           intruder_state_map,
                           asset_state);
    

    for(const auto &usv_state_id_pair : usv_state_map){
        if(swarm.contains_usv(usv_state_id_pair.first)){
            swarm.update_usv_state_estimate(usv_state_id_pair.second);
            ROS_INFO("Updating usv %d", usv_state_id_pair.first);
        }
        else{
            ROS_INFO("Adding new usv %d to swarm", usv_state_id_pair.first);

            std::string usv_head_str = (boost::format("/swarm_simulation/usv_params/usv_%d") % usv_state_id_pair.first).str();
            swarm.add_usv(agent::USVAgent(usv_state_id_pair.second,
                   usv_constraints_map[usv_state_id_pair.first],
                   usv_radar_params_map[usv_state_id_pair.first],
                   agent::AgentAssignment()));
        }
    }

    for(const auto &intruder_state_id_pair : intruder_state_map){
        if(swarm.contains_intruder(intruder_state_id_pair.first)){
            swarm.update_intruder_state_estimate(intruder_state_id_pair.second);
            ROS_INFO("Updating intruder %d", intruder_state_id_pair.first);
        }
        else{
            ROS_INFO("Adding new intruder %d", intruder_state_id_pair.first);
           swarm.add_intruder(agent::ObservedIntruderAgent(intruder_state_id_pair.second,
                   intruder_constraints_map[intruder_state_id_pair.first],
                   intruder_radar_params_map[intruder_state_id_pair.first]));
        }
    }

    for (const auto &task : swarm.get_swarm_assignment()[usv_id]){
        if (task.task_type != agent::TaskType::Guard && task.task_idx!=-1 && task.task_idx>100){
            const auto intruder = swarm.get_intruder_estimate_by_id(task.task_idx);
            double probability = intruder.get_threat_probability();
            bool classification = intruder.is_threat();
            publish_threat_statistics(task.task_idx, probability, classification);
        }
    }


    // swarm.update_intruder_estimates(intruder_state_map);

    agent::USVAgent usv = swarm.get_usv_estimate_by_id(usv_id);
    int sim_id = usv.get_sim_id();
    agent::AssetAgent asset = agent::AssetAgent(asset_state);

    agent::MotionGoal motion_goal;

    swarm_control::get_motion_goal_from_assignment(usv_id,
                                                   swarm,
                                                   motion_goal);

    publish_markers(motion_goal);

    agent::AgentCommand command;
    swarm_control::get_usv_command_from_motion_goal(usv_id,
                                                    swarm,
                                                    motion_goal,
                                                    command);
    ROS_INFO("DERSIRED COMMAND: (%f, %f)", command.delta_speed, command.delta_heading*180/swarm_tools::PI);

    std::vector<agent::AgentState> obstacle_states = swarm.get_obstacle_states();
    collision_avoidance::correct_command(usv,
                                         obstacle_states,
                                         command);
    ROS_INFO("CORRECTED COMMAND: (%f, %f)", command.delta_speed, command.delta_heading*180/swarm_tools::PI);

    swarm_msgs::agentCommand command_msg = swarm_msgs::agentCommand();
    command_msg.delta_heading = command.delta_heading;
    command_msg.delta_speed = command.delta_speed;
    command_msg.sim_id = usv.get_sim_id();
    command_pub.publish(command_msg);
} // callback


void task_allocator_callback(const swarm_msgs::swarmAssignment::ConstPtr &swarm_task_assignment_ptr){
    ROS_INFO("Setting new task allocation!");
    agent::SwarmAssignment swarm_assignment = extract_from_swarm_assignment_msg(*swarm_task_assignment_ptr);
    ROS_INFO("%s", agent::swarm_assignment_to_string(swarm_assignment).c_str());
    mtx.lock();
    try{
        swarm.update_swarm_assignment(swarm_assignment);
    }catch(std::exception &e){
        throw "Task Allocation Failed.";
    }
    mtx.unlock();
}

bool model_predictive_response(swarm_task_manager::modelPredictiveSimulation::Request &req,
                               swarm_task_manager::modelPredictiveSimulation::Response &res){
    if(swarm.get_num_usvs()==0){
        return false;
    }
    if(!mtx.try_lock()) return false;
    int num_timesteps_lookahead=req.num_timesteps_lookahead;
    double delta_time_secs = req.delta_time_secs;
    double threshold = req.threshold;
    agent::WeightedSwarmAssignment weighted_swarm_assignment;
    ROS_INFO("Model predictive request (%d, %f, %f)",
                num_timesteps_lookahead,
                delta_time_secs,
                threshold);
    try{
    weighted_swarm_assignment = swarm_task_manager::get_best_candidate_swarm_assignment(usv_id,
                                                                                             swarm,
                                                                                             usv_communication_map,
                                                                                             num_timesteps_lookahead,
                                                                                             delta_time_secs,
                                                                                             threshold);
 
    res.candidate_assignment = convert_to_swarm_assignment_msg(weighted_swarm_assignment.first);
    res.weight = weighted_swarm_assignment.second;
    }catch(std::exception& e){
        ROS_ERROR("Task Allocation ERROR: %s", e.what());
        mtx.unlock();
        return false;
    }
    auto swarm_assignment = weighted_swarm_assignment.first;
    ROS_INFO("Submitting Swarm Assignment");
    ROS_INFO(agent::swarm_assignment_to_string(swarm_assignment).c_str());
    ROS_INFO("WEIGHT: %f", weighted_swarm_assignment.second);
    ROS_INFO("Model Predictive Successfull");
    mtx.unlock();
    return true;
}

void reset_callback(){
    initialized=false;
}


int main(int argc, char **argv){

    ros_container_ptr.reset(new RosContainer(argc, argv, "usv_planner"));
    ros::NodeHandle nh_priv("~");
    if(!nh_priv.getParam("usv_id", usv_id)){
        ROS_ERROR("USV ID NOT FOUND");
    }
    else{ROS_INFO("Found USV ID %d IN PARAMETERS", usv_id);}
    // Initialize swarm
    swarm.set_main_usv_id(usv_id);
    auto threat_detection_client = ros_container_ptr->nh.serviceClient<swarm_threat_detection::ThreatDetection>("ThreatDetection");
    swarm.set_threat_detection_client(threat_detection_client);

    // Command and motion goal marker publisher
    command_pub = ros_container_ptr->nh.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    marker_pub = ros_container_ptr->nh.advertise<swarm_msgs::simulationMarker>("Markers", 1000);
    threat_pub = ros_container_ptr->nh.advertise<swarm_msgs::threatStatistics>("threatStatistics", 1000);

    // Perception Subscriber
    ros::Subscriber perc_sub = ros_container_ptr->nh.subscribe("Perception", 1000, callback);
    ros::Subscriber task_sub = ros_container_ptr->nh.subscribe("Task_Allocation", 1000, task_allocator_callback);
    
    // Advertise sync service
    auto sync_name = boost::format("sync_service_%d") % usv_id;
    auto mp_name = boost::format("mp_simulation_service_%d") % usv_id;
    ros::ServiceServer mp_service = ros_container_ptr->nh.advertiseService(mp_name.str(), model_predictive_response);

    // Get USV Parameters
    std::string usv_head_str = "/swarm_simulation/usv_params";
    get_agent_parameters(ros_container_ptr, usv_head_str, usv_constraints_map, usv_radar_params_map);

    // Get Intruder Parameters
    std::string intruder_head_str = "/swarm_simulation/intruder_params";
    get_agent_parameters(ros_container_ptr, intruder_head_str, intruder_constraints_map, intruder_radar_params_map);
    ros::spin();

    return 0;
}