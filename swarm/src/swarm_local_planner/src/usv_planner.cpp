#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/format.hpp>
//#include <boost/shared_ptr>
#include <mutex>
#include "agent.h"
#include "collision_avoidance.h"
#include "motion_goal_control.h"
#include "ros_swarm_tools.h"
#include "swarm_msgs/agentType.h"
#include "swarm_msgs/agentState.h"
#include "swarm_msgs/worldState.h"
#include "swarm_msgs/agentCommand.h"
#include "swarm_msgs/simulationMarker.h"
#include "swarm_local_planner/CollisionAvoidance.h"
#include "swarm_task_manager/resync.h"

agent::USVAgent usv;
int usv_id;
int intruder_id;
struct RosInit{
    RosInit(int argc, char** argv, const char *node_name){
        ros::init(argc, argv, node_name);
    }
};
struct RosContainer{

    RosInit init;
    ros::NodeHandle n;
    ros::Publisher command_pub;
    ros::Publisher marker_pub;
    ros::ServiceClient client;

    RosContainer(int argc, char** argv, const char *node_name) : init(argc, argv, node_name){}
};

// Declare ros container
boost::shared_ptr<RosContainer> ros_container_ptr;

std::map<int, ros::ServiceClient> usv_sync_service_map;
std::map<int, bool> usv_communication_map;
std::map<int, bool> usv_sync_map;

std::mutex mtx;

void publish_markers(const agent::MotionGoal &delay_motion_goal,
                     const agent::MotionGoal &guard_motion_goal,
                     const agent::MotionGoal &motion_goal){
    // Motion Goal 
    ROS_INFO("Publishing Delay Motion Goal ([%f], [%f])", delay_motion_goal.x, delay_motion_goal.y);
    swarm_msgs::simulationMarker delay_marker;
    delay_marker.x = delay_motion_goal.x;
    delay_marker.y = delay_motion_goal.y;
    delay_marker.sim_id = usv.get_sim_id()+400;
    delay_marker.colour = "GREEN";
    ros_container_ptr->marker_pub.publish(delay_marker);

    ROS_INFO("Publishing Guard Motion Goal ([%f], [%f])", guard_motion_goal.x, guard_motion_goal.y);
    swarm_msgs::simulationMarker guard_marker;
    guard_marker.x = guard_motion_goal.x;
    guard_marker.y = guard_motion_goal.y;
    guard_marker.sim_id = usv.get_sim_id()+500;
    guard_marker.colour = "YELLOW";
    ros_container_ptr->marker_pub.publish(guard_marker);

    ROS_INFO("Publishing Motion Goal ([%f], [%f])", motion_goal.x, motion_goal.y);
    swarm_msgs::simulationMarker marker;
    marker.x = motion_goal.x;
    marker.y = motion_goal.y;
    marker.sim_id = usv.get_sim_id()+300;
    marker.colour = "RED";
    ros_container_ptr->marker_pub.publish(marker);
}

int collision_avoidance_check(const int &sim_id,
                              const agent::AgentConstraints constraints,
                              const double &max_distance,
                              const double &max_angle_rad,
                              const double &aggression,
                              const swarm_msgs::worldStateConstPtr &world_state,
                              const agent::AgentCommand &command,
                              swarm_local_planner::CollisionAvoidance &srv){

    // WorldState
    srv.request.world_state = *world_state;
    
    // Command
    srv.request.desired_command.sim_id=sim_id;
    srv.request.desired_command.delta_speed=command.delta_speed;
    srv.request.desired_command.delta_heading=command.delta_heading;
    
    // Constraints
    srv.request.agent_constraints.sim_id=sim_id;
    srv.request.agent_constraints.max_speed;
    srv.request.agent_constraints.max_delta_speed;
    srv.request.agent_constraints.max_delta_heading;

    // Parameters
    srv.request.agent_params.sim_id=sim_id;
    srv.request.agent_params.max_distance=max_distance;
    srv.request.agent_params.max_angle=max_angle_rad;
    srv.request.agent_params.aggression=aggression;

    ROS_INFO("Correcting command for USV [%d]", sim_id);
    ROS_INFO("Recommended command ([%f], [%f])",
            command.delta_speed,
            command.delta_heading*180/swarm_tools::PI);
    if (ros_container_ptr->client.call(srv)){
        ROS_INFO("Collision Avoidance Service successful.");
        ROS_INFO("Corrected command ([%f], [%f])",
                srv.response.safe_command.delta_speed,
                srv.response.safe_command.delta_heading*180/swarm_tools::PI);
        return 0;
    }else{
        ROS_ERROR("Collision Avoidance Service Failed!");
        return -1;
    }
}

bool get_motion_goals_from_assignment(const int &num_usvs,
                                      const agent::USVAgent &usv,
                                      const agent::AssetAgent &asset,
                                      agent::MotionGoal &delay_motion_goal,
                                      agent::MotionGoal &guard_motion_goal,
                                      agent::MotionGoal &motion_goal
                                      ){
    agent::AgentAssignment assignment = usv.get_current_assignment();
    if (assignment.guard_assignment_idx == -1){
        int guard_index = assignment.guard_assignment_idx;
        ROS_INFO("Assignment: Delay index: NULL Guard index %d", guard_index);
        swarm_control::usv_guard_motion_goal(num_usvs,
                                             guard_index,
                                             100,
                                             asset.get_state(),
                                             motion_goal
                                             );


    } else if(assignment.delay_assignment_idx == -1){
        int intruder_id = assignment.delay_assignment_idx;
        ROS_INFO("Assignment: Delay index: %d Guard index: NULL", intruder_id);
        agent::IntruderAgent intruder = usv.get_intruder_estimate_by_id(intruder_id);
        swarm_control::usv_delay_motion_goal(usv,
                                             intruder,
                                             asset,
                                             motion_goal);
    }
    else{
        int guard_index = assignment.guard_assignment_idx;
        int intruder_id = assignment.delay_assignment_idx;
        ROS_INFO("Assignment: Delay index: %d Guard index: %d", intruder_id, guard_index);
        agent::IntruderAgent intruder = usv.get_intruder_estimate_by_id(intruder_id);
        
        swarm_control::usv_delay_motion_goal(usv,
                                             intruder,
                                             asset,
                                             delay_motion_goal);
        
        swarm_control::usv_guard_motion_goal(num_usvs,
                                             guard_index,
                                             100,
                                             asset.get_state(),
                                             guard_motion_goal
                                             );
        std::vector<agent::MotionGoal> mgs = {delay_motion_goal, guard_motion_goal};
        double dist = swarm_tools::euclidean_distance(intruder.get_position(), asset.get_position());
        double alpha = dist/1000;
        if (dist>1000){
            alpha = 0.7;
        }
        std::vector<double> weights = {1-alpha, alpha};
        swarm_control::weighted_motion_goal(mgs,
                                            weights,
                                            motion_goal);
    return true;
    }
}


void callback(const swarm_msgs::worldState::ConstPtr& world_state){
    std::map<int, agent::AgentState> usv_state_map;
    std::map<int, agent::AgentState> intruder_state_map;
    agent::AgentState asset_state;

    extract_from_world_msg(world_state,
                           usv_state_map,
                           intruder_state_map,
                           asset_state);

    usv.update_estimates(usv_state_map, intruder_state_map);
    ROS_INFO("SIM ID: %d SPEED: %f X: %f Y: %f",
             usv.get_sim_id(),
             usv.get_speed(),
             usv.get_x(),
             usv.get_y());

    int num_usvs = usv_state_map.size();
    int sim_id = usv.get_sim_id();
    agent::AssetAgent asset = agent::AssetAgent(asset_state);

    agent::MotionGoal motion_goal;
    agent::MotionGoal guard_motion_goal;
    agent::MotionGoal delay_motion_goal;

    get_motion_goals_from_assignment(num_usvs,
                                     usv,
                                     asset,
                                     delay_motion_goal,
                                     guard_motion_goal,
                                     motion_goal);

    publish_markers(delay_motion_goal,
                    guard_motion_goal,
                    motion_goal);

    agent::AgentCommand command;
    swarm_control::move_to_motion_goal(usv.get_state(),
                                       usv.get_constraints(),
                                       motion_goal,
                                       command);

    // double max_distance = 40;
    // double max_angle_rad = swarm_tools::PI/2;
    // double aggression = 0.999;
    // swarm_msgs::agentCommand na_command;
    // na_command.sim_id = usv_id;
    // na_command.delta_heading = command.delta_heading;
    // na_command.delta_speed = command.delta_speed;
    // command_pub.publish(na_command);
    // return;

    collision_avoidance::correct_command(usv,
                                         usv.get_obstacle_states(),
                                         command);
    // swarm_local_planner::CollisionAvoidance srv;
    // collision_avoidance_check(sim_id,
    //                           usv.get_constraints(),
    //                           max_distance,
    //                           max_angle_rad,
    //                           aggression,
    //                           world_state,
    //                           command,
    //                           srv);
    // command_pub.publish(srv.request.desired_command);
    swarm_msgs::agentCommand command_msg = swarm_msgs::agentCommand();
    command_msg.delta_heading = command.delta_heading;
    command_msg.delta_speed = command.delta_speed;
    command_msg.sim_id = usv.get_sim_id();
    ros_container_ptr->command_pub.publish(command_msg);

} // callback


bool sync_response(swarm_task_manager::resync::Request &req,
                   swarm_task_manager::resync::Response &res){

    // check if you have already synced and you have communication
    if (usv_sync_map[req.sim_id] && usv_communication_map[req.sim_id]){
        return false;
    }

    mtx.lock();
    usv_sync_map[req.sim_id] = true;
    usv_communication_map[req.sim_id] = true;
    mtx.unlock();

    std::map<int, agent::IntruderAgent> intruders = extract_from_intruder_msgs(req.intruders);
    std::map<int, agent::USVAgent> usvs = extract_from_usv_msgs(req.usvs);
    usv.update_estimates(usvs, intruders);
    // TODO: Do something with this
}

bool sync_request(int response_sim_id,
                  swarm_task_manager::resync::Request &req,
                  swarm_task_manager::resync::Response &res){
    
    // check if you have already synced and you have communication
    if (usv_sync_map[response_sim_id] && usv_communication_map[response_sim_id]){
        return false;
    }

    auto index = usv_sync_service_map.find(response_sim_id);

    if (index == usv_sync_service_map.end()){
        auto sync_name = boost::format("sync_service_%d") % usv_id;
        usv_sync_service_map[response_sim_id] = ros_container_ptr->n.serviceClient<swarm_task_manager::resync>(sync_name.str()); 
    }
    auto sync_client = usv_sync_service_map[response_sim_id];
    if(sync_client.call(req, res)){
        mtx.lock();
        usv_sync_map[response_sim_id] = true;
        usv_communication_map[response_sim_id] = true;
        mtx.unlock();

        // TODO: do stuff here
    }else{
        mtx.lock();
        usv_sync_map[response_sim_id] = true;
        usv_communication_map[response_sim_id] = false;
        mtx.unlock();
    }
}

bool submit_weighted_assignments(){

}

int main(int argc, char **argv){

    if (argc >2){
        usv_id = std::stoi(argv[1]);
        intruder_id = std::stoi(argv[2]);
    }else{
        usv_id = 1;
        intruder_id = 101;
    }
    auto s = boost::format("usv_planner_%d") % usv_id;
    ros_container_ptr.reset(new RosContainer(argc, argv, s.str().c_str()));
    // Initialize usv
    usv = agent::USVAgent(agent::AgentState {0, 0, 0, 0, 30, usv_id});


    // Command and motion goal marker publisher
    ros_container_ptr->command_pub = ros_container_ptr->n.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    ros_container_ptr->marker_pub = ros_container_ptr->n.advertise<swarm_msgs::simulationMarker>("Markers", 1000);
    
    // Perception Subscriber
    ros::Subscriber sub = ros_container_ptr->n.subscribe("Perception", 1000, callback);
    // client = n.serviceClient<swarm_local_planner::CollisionAvoidance>("collision_avoidance");
    
    // Advertise sync service
    auto sync_name = boost::format("sync_service_%d") % usv_id;
    ros::ServiceServer sync_service = ros_container_ptr->n.advertiseService(sync_name.str(), sync_response);
    ros::spin();
    return 0;

    //ros::Rate loop_rate(10);
    // loop_rate.sleep();

}