#include <vector>
#include <ros/ros.h>
#include "agent.h"
#include "collision_avoidance.h"
#include "motion_goal_control.h"
#include "ros_swarm_tools.h"
#include "swarm_msgs/agentType.h"
#include "swarm_msgs/agentState.h"
#include "swarm_msgs/worldState.h"
#include "swarm_msgs/agentCommand.h"
#include "swarm_control/CollisionAvoidance.h"

ros::Publisher command_pub;
ros::ServiceClient client;
RosContainerPtr ros_container_ptr;

void callback(const swarm_msgs::worldState::ConstPtr& world_state){
    auto ws = world_state->worldState;

    std::vector<agent::AgentState> intruders;
    agent::AgentState tanker;
    for ( swarm_msgs::agentState agent_state : ws){
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

            agent::AgentState intruder {x, y, speed, heading, radius, sim_id}; 
            intruders.push_back(intruder);
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

    for (auto intruder : intruders){
        int sim_id = intruder.sim_id;
        double max_speed = 35;
        double max_delta_speed = 5; 
        double max_delta_heading =  swarm_tools::PI/2;
        agent::AgentConstraints constraints;
        constraints.max_speed = max_speed;
        constraints.max_delta_speed = max_delta_speed;
        constraints.max_delta_heading = max_delta_heading;
        agent::AgentCommand command;

        agent::MotionGoal motion_goal = {tanker.x, tanker.y};
        
        ROS_INFO("Motion Goal ([%f], [%f])", motion_goal.x, motion_goal.y);

        swarm_control::get_command_from_motion_goal(intruder,
                                                    constraints,
                                                    motion_goal,
                                                    command);

        std::string intruder_head_str = (boost::format("/swarm_simulation/intruder_params/intruder_%d") % sim_id).str();
        agent::AgentConstraints intruder_constraints;
        agent::CollisionAvoidanceParameters intruder_radar_params;
        get_agent_parameters(ros_container_ptr, intruder_head_str, intruder_constraints, intruder_radar_params);
       
        swarm_control::CollisionAvoidance srv;
        
        // WorldState
        srv.request.world_state = *world_state;
        
        // Command
        srv.request.desired_command.sim_id=sim_id;
        srv.request.desired_command.delta_speed=command.delta_speed;
        srv.request.desired_command.delta_heading=command.delta_heading;
        
        // Constraints
        srv.request.agent_constraints.sim_id=sim_id;
        srv.request.agent_constraints.max_speed=intruder_constraints.max_speed;
        srv.request.agent_constraints.max_delta_speed=intruder_constraints.max_delta_speed;
        srv.request.agent_constraints.max_delta_heading=intruder_constraints.max_delta_heading;

        // Parameters
        srv.request.agent_params.sim_id=sim_id;
        srv.request.agent_params.max_distance=intruder_radar_params.max_radar_distance;
        srv.request.agent_params.max_angle=intruder_radar_params.max_radar_angle_rad;
        srv.request.agent_params.aggression=intruder_radar_params.aggression;

        ROS_INFO("Correcting command for intruder [%d]", sim_id);
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
}

int main(int argc, char **argv){
    ros_container_ptr.reset(new RosContainer(argc, argv, "intruder_control"));
    command_pub = ros_container_ptr->nh.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    ros::Subscriber sub = ros_container_ptr->nh.subscribe("Perception", 1000, callback);
    client = ros_container_ptr->nh.serviceClient<swarm_control::CollisionAvoidance>("collision_avoidance");
    ros::spin();
    return 0;
}