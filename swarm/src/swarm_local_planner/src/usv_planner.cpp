#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/format.hpp>
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

ros::Publisher command_pub;
ros::Publisher marker_pub;
ros::ServiceClient client;
agent::USVAgent usv;
int usv_id;
int intruder_id;

void publish_markers(agent::MotionGoal delay_motion_goal,
                     agent::MotionGoal guard_motion_goal,
                     agent::MotionGoal motion_goal){
    // Motion Goal 
    ROS_INFO("Publishing Delay Motion Goal ([%f], [%f])", delay_motion_goal.x, delay_motion_goal.y);
    swarm_msgs::simulationMarker delay_marker;
    delay_marker.x = delay_motion_goal.x;
    delay_marker.y = delay_motion_goal.y;
    delay_marker.sim_id = usv.get_sim_id()+400;
    delay_marker.colour = "GREEN";
    marker_pub.publish(delay_marker);

    ROS_INFO("Publishing Guard Motion Goal ([%f], [%f])", guard_motion_goal.x, guard_motion_goal.y);
    swarm_msgs::simulationMarker guard_marker;
    guard_marker.x = guard_motion_goal.x;
    guard_marker.y = guard_motion_goal.y;
    guard_marker.sim_id = usv.get_sim_id()+500;
    guard_marker.colour = "YELLOW";
    marker_pub.publish(guard_marker);

    ROS_INFO("Publishing Motion Goal ([%f], [%f])", motion_goal.x, motion_goal.y);
    swarm_msgs::simulationMarker marker;
    marker.x = motion_goal.x;
    marker.y = motion_goal.y;
    marker.sim_id = usv.get_sim_id()+300;
    marker.colour = "RED";
    marker_pub.publish(marker);
}

int collision_avoidance_check(int sim_id,
                              agent::AgentConstraints constraints,
                              double max_distance,
                              double max_angle_rad,
                              double aggression,
                              swarm_msgs::worldStateConstPtr world_state,
                              agent::AgentCommand command,
                              swarm_local_planner::CollisionAvoidance srv){

    // WorldState
    srv.request.world_state = *world_state;
    
    // Command
    srv.request.desired_command.sim_id=sim_id;
    srv.request.desired_command.delta_speed=command.delta_speed;
    srv.request.desired_command.delta_heading=command.delta_heading;
    
    // Constraints
    srv.request.agent_constraints.sim_id=sim_id;
    srv.request.agent_constraints.max_speed=constraints.max_speed;
    srv.request.agent_constraints.max_delta_speed=constraints.max_delta_speed;
    srv.request.agent_constraints.max_delta_heading=constraints.max_delta_heading;

    // Parameters
    srv.request.agent_params.sim_id=sim_id;
    srv.request.agent_params.max_distance=max_distance;
    srv.request.agent_params.max_angle=max_angle_rad;
    srv.request.agent_params.aggression=aggression;

    ROS_INFO("Correcting command for USV [%d]", sim_id);
    ROS_INFO("Recommended command ([%f], [%f])",
            command.delta_speed,
            command.delta_heading*180/swarm_tools::PI);
    if (client.call(srv)){
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



void callback(const swarm_msgs::worldState::ConstPtr& world_state){
    std::map<int, agent::AgentState> usv_state_map;
    std::map<int, agent::AgentState> intruder_state_map;
    agent::AgentState asset_state;

    extract_from_world_msg(world_state,
                           usv_state_map,
                           intruder_state_map,
                           asset_state);

    int num_usvs = usv_state_map.size();
    int sim_id = usv.get_sim_id();
    agent::AgentAssignment assignment = usv.current_assignment;
    agent::AssetAgent asset = agent::default_asset_agent(asset_state);
    agent::IntruderAgent intruder = usv.get_intruder_estimate_by_id(intruder_id);

    agent::MotionGoal motion_goal;
    agent::MotionGoal guard_motion_goal;
    agent::MotionGoal delay_motion_goal;
    if (assignment.guard_assignment_idx_ptr == nullptr){
        int guard_index = *assignment.guard_assignment_idx_ptr;
        swarm_control::usv_guard_motion_goal(num_usvs,
                                             guard_index,
                                             100,
                                             asset_state,
                                             motion_goal
                                             );


    } else if(assignment.delay_assignment_idx_ptr==nullptr){
        int intruder_id = *assignment.delay_assignment_idx_ptr;
        agent::IntruderAgent intruder = usv.get_intruder_estimate_by_id(intruder_id);
        swarm_control::usv_delay_motion_goal(usv,
                                            intruder,
                                            asset,
                                            delay_motion_goal);
    }
    else{
        int guard_index = *assignment.guard_assignment_idx_ptr;
        int intruder_id = *assignment.delay_assignment_idx_ptr;
        
        swarm_control::usv_delay_motion_goal(usv,
                                             intruder,
                                             asset,
                                             delay_motion_goal);
        
        swarm_control::usv_guard_motion_goal(num_usvs,
                                             guard_index,
                                             100,
                                             asset_state,
                                             guard_motion_goal
                                             );
        std::vector<agent::MotionGoal> mgs = {delay_motion_goal, guard_motion_goal};
        double dist = swarm_tools::euclidean_distance(intruder.position(), asset_state.position());
        double alpha = dist/1000;
        if (dist>1000){
            alpha = 0.7;
        }
        std::vector<double> weights = {1-alpha, alpha};
        swarm_control::weighted_motion_goal(mgs,
                                            weights,
                                            motion_goal);
    }

    publish_markers(delay_motion_goal,
                    guard_motion_goal,
                    motion_goal);

    agent::AgentCommand command;
    swarm_control::move_to_motion_goal(usv.state,
                                       usv.constraints,
                                       motion_goal,
                                       command);

    double max_distance = 40;
    double max_angle_rad = swarm_tools::PI/2;
    double aggression = 0.999;
    swarm_msgs::agentCommand na_command;
    na_command.sim_id = usv_id;
    na_command.delta_heading = command.delta_heading;
    na_command.delta_speed = command.delta_speed;
    // command_pub.publish(na_command);
    // return;

    swarm_local_planner::CollisionAvoidance srv;
    collision_avoidance_check(sim_id,
                              usv.constraints,
                              max_distance,
                              max_angle_rad,
                              aggression,
                              world_state,
                              command,
                              srv
    );

    command_pub.publish(srv.request.desired_command);

} // callback



int main(int argc, char **argv){

    if (argc >2){
        usv_id = std::stoi(argv[1]);
        intruder_id = std::stoi(argv[2]);
    }else{
        usv_id = 1;
        intruder_id = 101;
    }
    auto s = boost::format("usv_planner_%d") % usv_id;

    usv = agent::default_usv_agent(agent::AgentState {0,0,0});
    
    ros::init(argc, argv, s.str());
    ros::NodeHandle n;
    command_pub = n.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    marker_pub = n.advertise<swarm_msgs::simulationMarker>("Markers", 1000);
    ros::Subscriber sub = n.subscribe("Perception", 1000, callback);
    client = n.serviceClient<swarm_local_planner::CollisionAvoidance>("collision_avoidance");
    ros::spin();

    return 0;
    //ros::Rate loop_rate(10);
    // loop_rate.sleep();
}