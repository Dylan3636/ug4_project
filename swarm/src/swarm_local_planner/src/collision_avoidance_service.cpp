#include <vector>
#include <ros/ros.h>
#include "agent.h"
#include "collision_avoidance.h"
#include "swarm_local_planner/CollisionAvoidance.h"
#include "swarm_msgs/worldState.h"
#include "swarm_msgs/agentCommand.h"


bool avoid_collision(swarm_local_planner::CollisionAvoidance::Request &req,
                     swarm_local_planner::CollisionAvoidance::Response &res)
{
    ROS_INFO("I AM BEING CALLED!");
    // std::vector<swarm_msgs::agentState_<std::allocator<void>>, std::allocator<swarm_msgs::agentState_<std::allocator<void>>>> ws = msg->worldState;
    auto ws = req.world_state.worldState;
    std::vector<agent::AgentState> obstacles;
    agent::AgentState usv;

    int usv_id = req.desired_command.sim_id;
    for (auto agent_state : ws){

        int sim_id = agent_state.sim_id;
        double x = agent_state.x;
        double y = agent_state.y;
        double speed = agent_state.speed;
        double heading = agent_state.heading;
        double radius = agent_state.radius;

        // ROS_INFO("I heard from agent [%d]: \nx: [%f] \ny: [%f] \nspeed: [%f] \nheading: [%f], \nradius: [%f]",
        //         sim_id,
        //         x,
        //         y,
        //         speed,
        //         heading*180/swarm_tools::PI,
        //         radius
        //         );
        if (sim_id==usv_id){
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

    agent::AgentCommand command = {req.desired_command.delta_speed,
                                   req.desired_command.delta_heading};


    agent::AgentConstraints constraints = {req.agent_constraints.max_speed,
                                           req.agent_constraints.max_delta_speed,
                                           req.agent_constraints.max_delta_heading};

    double max_distance = req.agent_params.max_distance;
    double max_angle_rad = req.agent_params.max_angle;
    double aggression = req.agent_params.aggression;

    int flag = collision_avoidance::correct_command(usv,
                                        command,
                                        edges,
                                        constraints,
                                        max_distance,
                                        max_angle_rad,
                                        aggression);
    ROS_INFO("Correct command [%d]", flag);

    
    res.safe_command.sim_id = usv.sim_id;
    res.safe_command.delta_speed = command.delta_speed;
    res.safe_command.delta_heading = command.delta_heading;
    return true;
    //command_pub.publish(command_msg);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "collision_avoidance_service");

    ros::NodeHandle n;
    //command_pub = n.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    ros::ServiceServer service = n.advertiseService("collision_avoidance", avoid_collision);
    ros::spin();
    return 0;
    //ros::Rate loop_rate(10);
    // loop_rate.sleep();
}