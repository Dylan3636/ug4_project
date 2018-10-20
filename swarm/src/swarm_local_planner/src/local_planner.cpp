#include <vector>
#include <ros/ros.h>
#include "agent.h"
#include "collision_avoidance.h"
#include "swarm_local_planner/CollisionAvoidance.h"
#include "swarm_msgs/worldState.h"
#include "swarm_msgs/agentCommand.h"

ros::Publisher command_pub;

void callback(const swarm_msgs::worldState::ConstPtr& msg){
    std::vector<swarm_msgs::agentState_<std::allocator<void>>, std::allocator<swarm_msgs::agentState_<std::allocator<void>>>> ws = msg->worldState;
    std::vector<agent::AgentState> obstacles;
    agent::AgentState usv;
    for (auto agent_state : ws){

        int sim_id = agent_state.sim_id;
        double x = agent_state.x;
        double y = agent_state.y;
        double speed = agent_state.speed;
        double heading = agent_state.heading;
        double radius = agent_state.radius;

        ROS_INFO("I heard from agent [%d]: \nx: [%f] \ny: [%f] \nspeed: [%f] \nheading: [%f], \nradius: [%f]",
                sim_id,
                x,
                y,
                speed,
                heading,
                radius
                );
        if (sim_id==0){
           usv = {x, y, speed, heading, radius, sim_id}; 
        }
        else{
            agent::AgentState as = {x, y, speed, heading, radius, sim_id};
            obstacles.push_back(as);
        }
    }
    std::vector<swarm_tools::PointInterval> edges;
    for (auto obstacle : obstacles){
        swarm_tools::Point2D left_edge;
        swarm_tools::Point2D right_edge;
        swarm_tools::edge_points_of_circle(usv.position(),
                                           obstacle.position(),
                                           obstacle.radius,
                                           left_edge,
                                           right_edge
                                           );
        swarm_tools::PointInterval pi = {left_edge, right_edge};
        edges.push_back(pi);
    }

    agent::AgentCommand command = {0, 0};
    agent::AgentConstraints constraints = {10, 1, swarm_tools::PI/4};
    double max_distance = 400;
    double max_angle_rad = swarm_tools::PI/3;
    double aggression = 0.5;

    collision_avoidance::correct_command(usv,
                                        command,
                                        edges,
                                        constraints,
                                        max_distance,
                                        max_angle_rad,
                                        aggression);
    
    swarm_msgs::agentCommand command_msg;
    command_msg.sim_id = usv.sim_id;
    command_msg.delta_speed = command.delta_speed;
    command_msg.delta_heading = command.delta_heading;
    command_pub.publish(command_msg);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "planner");

    ros::NodeHandle n;
    command_pub = n.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    ros::Subscriber sub = n.subscribe("Perception", 1000, callback);
    ros::spin();
    //ros::Rate loop_rate(10);
    // loop_rate.sleep();
}
