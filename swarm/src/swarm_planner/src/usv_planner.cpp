// #include <string>
// #include <ros/ros.h>
// #include <boost/format.hpp>
// #include <boost/shared_ptr>
#include <mutex>
#include "motion_goal_control.h"
#include "collision_avoidance.h"
#include "ros_swarm_tools.h"
#include "swarm_msgs/simulationMarker.h"
#include "task_allocation.h"
// #include "swarm_task_manager/resync.h"
#include "swarm_task_manager/modelPredictiveSimulation.h"

bool initialized = false;
agent::USVSwarm swarm;
int usv_id;
int intruder_id;


// // Declare ros container
RosContainerPtr ros_container_ptr;
ros::Publisher command_pub;
ros::Publisher marker_pub;
ros::ServiceClient client;

std::map<int, ros::ServiceClient> usv_sync_service_map;
std::map<int, bool> usv_communication_map;
std::map<int, bool> usv_sync_map;
std::map<int, agent::AgentState> usv_state_map;
std::map<int, agent::AgentState> intruder_state_map;
agent::AgentState asset_state;

std::mutex mtx;

void publish_markers(const agent::MotionGoal &delay_motion_goal,
                     const agent::MotionGoal &guard_motion_goal,
                     const agent::MotionGoal &motion_goal){
    // Motion Goal 
    ROS_INFO("Publishing Delay Motion Goal ([%f], [%f])", delay_motion_goal.x, delay_motion_goal.y);
    swarm_msgs::simulationMarker delay_marker;
    delay_marker.x = delay_motion_goal.x;
    delay_marker.y = delay_motion_goal.y;
    delay_marker.sim_id = usv_id+400;
    delay_marker.colour = "GREEN";
    marker_pub.publish(delay_marker);

    ROS_INFO("Publishing Guard Motion Goal ([%f], [%f])", guard_motion_goal.x, guard_motion_goal.y);
    swarm_msgs::simulationMarker guard_marker;
    guard_marker.x = guard_motion_goal.x;
    guard_marker.y = guard_motion_goal.y;
    guard_marker.sim_id = usv_id+500;
    guard_marker.colour = "YELLOW";
    marker_pub.publish(guard_marker);

    ROS_INFO("Publishing Motion Goal ([%f], [%f])", motion_goal.x, motion_goal.y);
    swarm_msgs::simulationMarker marker;
    marker.x = motion_goal.x;
    marker.y = motion_goal.y;
    marker.sim_id = usv_id+300;
    marker.colour = "RED";
    marker_pub.publish(marker);
}

void callback(const swarm_msgs::worldState::ConstPtr& world_state_ptr){

    ROS_INFO("USV Callback");
    usv_state_map.clear();
    intruder_state_map.clear();

    extract_from_world_msg(world_state_ptr,
                           usv_state_map,
                           intruder_state_map,
                           asset_state);
 
    swarm.update_estimates(usv_state_map, intruder_state_map);

    agent::USVAgent usv = swarm.get_usv_estimate_by_id(usv_id);
    int num_usvs = usv_state_map.size();
    int sim_id = usv.get_sim_id();
    agent::AssetAgent asset = agent::AssetAgent(asset_state);

    agent::MotionGoal motion_goal;
    agent::MotionGoal guard_motion_goal;
    agent::MotionGoal delay_motion_goal;

    swarm_control::get_motion_goals_from_assignment(usv_id,
                                                    swarm,
                                                    asset,
                                                    delay_motion_goal,
                                                    guard_motion_goal,
                                                    motion_goal);

    publish_markers(delay_motion_goal,
                    guard_motion_goal,
                    motion_goal);

    agent::AgentCommand command;
    swarm_control::get_command_from_motion_goal(usv.get_state(),
                                                usv.get_constraints(),
                                                motion_goal,
                                                command);

    std::vector<agent::AgentState> obstacle_states = swarm.get_obstacle_states();
    collision_avoidance::correct_command(usv,
                                         obstacle_states,
                                         command);

    swarm_msgs::agentCommand command_msg = swarm_msgs::agentCommand();
    command_msg.delta_heading = command.delta_heading;
    command_msg.delta_speed = command.delta_speed;
    command_msg.sim_id = usv.get_sim_id();
    command_pub.publish(command_msg);
} // callback


// bool sync_response(swarm_task_manager::resync::Request &req,
//                    swarm_task_manager::resync::Response &res){

//     // check if you have already synced and you have communication
//     if (usv_sync_map[req.sim_id] && usv_communication_map[req.sim_id]){
//         return false;
//     }

//     mtx.lock();
//     usv_sync_map[req.sim_id] = true;
//     usv_communication_map[req.sim_id] = true;
//     mtx.unlock();

//     std::map<int, agent::IntruderAgent> intruders = extract_from_intruder_msgs(req.intruders);
//     std::map<int, agent::USVAgent> usvs = extract_from_usv_msgs(req.usvs);
//     swarm.update_estimates(usvs, intruders);
//     // TODO: Do something with this
// }

// bool sync_request(int response_sim_id,
//                   swarm_task_manager::resync::Request &req,
//                   swarm_task_manager::resync::Response &res){
    
//     // check if you have already synced and you have communication
//     if (usv_sync_map[response_sim_id] && usv_communication_map[response_sim_id]){
//         return false;
//     }

//     auto index = usv_sync_service_map.find(response_sim_id);

//     if (index == usv_sync_service_map.end()){
//         auto sync_name = boost::format("sync_service_%d") % usv_id;
//         usv_sync_service_map[response_sim_id] = ros_container_ptr->n.serviceClient<swarm_task_manager::resync>(sync_name.str()); 
//     }
//     auto sync_client = usv_sync_service_map[response_sim_id];
//     if(sync_client.call(req, res)){
//         mtx.lock();
//         usv_sync_map[response_sim_id] = true;
//         usv_communication_map[response_sim_id] = true;
//         mtx.unlock();

//         // TODO: do stuff here
//     }else{
//         mtx.lock();
//         usv_sync_map[response_sim_id] = true;
//         usv_communication_map[response_sim_id] = false;
//         mtx.unlock();
//     }
// }

void task_allocator_callback(swarm_msgs::swarmAssignment::ConstPtr swarm_task_assignment_ptr){
    ROS_INFO("Setting new task allocation!");
    agent::SwarmAssignment swarm_allocation = extract_from_swarm_assignment_msg(*swarm_task_assignment_ptr);
    mtx.lock();
    try{
        swarm.update_swarm_assignment(swarm_allocation);
    }catch(std::exception e){
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
    ROS_INFO("Model predictive request (%d, %f, %f)",
                num_timesteps_lookahead,
                delta_time_secs,
                threshold);
    try{
    auto weighted_swarm_assignment = swarm_task_manager::get_best_candidate_swarm_assignment(usv_id,
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
    ROS_INFO("Model Predictive Successfull");
    mtx.unlock();
    return true;
}

int main(int argc, char **argv){

    usv_communication_map[1]=true;
    usv_communication_map[2]=true;

    ros_container_ptr.reset(new RosContainer(argc, argv, "usv_planner"));
    ros::NodeHandle nh_priv("~");
    if(!nh_priv.getParam("usv_id", usv_id)){
        ROS_ERROR("USV ID NOT FOUND");
        // throw std::exception("USV ID NOT FOUND");
    }
    else{ROS_INFO("Found USV ID %d IN PARAMETERS", usv_id);}
    // Initialize swarm
    // swarm();
    // usv = agent::USVAgent(agent::AgentState {0, 0, 0, 0, 30, usv_id});

    // Command and motion goal marker publisher
    command_pub = ros_container_ptr->nh.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    marker_pub = ros_container_ptr->nh.advertise<swarm_msgs::simulationMarker>("Markers", 1000);
    
    // Perception Subscriber
    ros::Subscriber perc_sub = ros_container_ptr->nh.subscribe("Perception", 1000, callback);
    ros::Subscriber task_sub = ros_container_ptr->nh.subscribe("Task_Allocation", 1000, task_allocator_callback);
    
    // Advertise sync service
    auto sync_name = boost::format("sync_service_%d") % usv_id;
    auto mp_name = boost::format("mp_simulation_service_%d") % usv_id;
    ///ros::ServiceServer sync_service = ros_container_ptr->n.advertiseService(sync_name.str(), sync_response);
    ros::ServiceServer mp_service = ros_container_ptr->nh.advertiseService(mp_name.str(), model_predictive_response);
    ros::spin();

    return 0;

    // ros::Rate loop_rate(10);
    // loop_rate.sleep();

}