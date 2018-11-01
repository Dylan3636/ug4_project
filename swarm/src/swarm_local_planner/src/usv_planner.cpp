#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/format.hpp>
#include "agent.h"
#include "collision_avoidance.h"
#include "motion_goal_control.h"
#include "swarm_msgs/agentType.h"
#include "swarm_msgs/agentState.h"
#include "swarm_msgs/worldState.h"
#include "swarm_msgs/agentCommand.h"
#include "swarm_msgs/simulationMarker.h"
#include "swarm_local_planner/CollisionAvoidance.h"

ros::Publisher command_pub;
ros::Publisher marker_pub;
ros::ServiceClient client;
int usv_id;
int intruder_id;

void callback(const swarm_msgs::worldState::ConstPtr& world_state){
    auto ws = world_state->worldState;

    agent::AgentState usv;
    agent::AgentState intruder;
    agent::AgentState tanker;
    for ( swarm_msgs::agentState agent_state : ws){
        if (agent_state.sim_id == usv_id){
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

            usv = {x, y, speed, heading, radius, sim_id}; 
        }
        if (agent_state.sim_id == intruder_id){
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

            intruder = {x, y, speed, heading, radius, sim_id}; 
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
            tanker = {x, y, speed, heading, radius, sim_id}; 
        }
    }

    int sim_id = usv.sim_id;
    double max_speed = 35;
    double max_delta_speed = 5; 
    double max_delta_heading =  swarm_tools::PI/2;
    agent::AgentConstraints constraints;
    constraints.max_speed = max_speed;
    constraints.max_delta_speed = max_delta_speed;
    constraints.max_delta_heading = max_delta_heading;
    double intruder_max_speed = 30;
    double intruder_max_delta_speed = 5; 
    double intruder_max_delta_heading =  swarm_tools::PI/6;
    agent::AgentConstraints intruder_constraints;
    intruder_constraints.max_speed = intruder_max_speed;
    intruder_constraints.max_delta_speed = intruder_max_delta_speed;
    intruder_constraints.max_delta_heading = intruder_max_delta_heading;
    agent::AgentCommand command;

    agent::MotionGoal motion_goal;
    swarm_control::usv_delay_motion_goal(usv,
                                         constraints,
                                         // 20,
                                         intruder,
                                         intruder_constraints,
                                         // 40,
                                         tanker,
                                         motion_goal);

    
    ROS_INFO("Motion Goal ([%f], [%f])", motion_goal.x, motion_goal.y);
    swarm_msgs::simulationMarker marker;
    marker.x = motion_goal.x;
    marker.y = motion_goal.y;
    marker.sim_id = usv.sim_id+300;
    marker_pub.publish(marker);

    swarm_control::move_to_motion_goal(usv,
                                       constraints,
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

    // WorldState
    srv.request.world_state = *world_state;
    
    // Command
    srv.request.desired_command.sim_id=sim_id;
    srv.request.desired_command.delta_speed=command.delta_speed;
    srv.request.desired_command.delta_heading=command.delta_heading;
    
    // Constraints
    srv.request.agent_constraints.sim_id=sim_id;
    srv.request.agent_constraints.max_speed=max_speed;
    srv.request.agent_constraints.max_delta_speed=max_delta_speed;
    srv.request.agent_constraints.max_delta_heading=max_delta_heading;

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
        command_pub.publish(srv.response.safe_command);
        ROS_INFO("Corrected command ([%f], [%f])",
                srv.response.safe_command.delta_speed,
                srv.response.safe_command.delta_heading*180/swarm_tools::PI);
    }else{
        ROS_ERROR("Collision Avoidance Service Failed!");
    }
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