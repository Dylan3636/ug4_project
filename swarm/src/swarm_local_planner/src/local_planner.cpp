#include <vector>
#include <ros/ros.h>
#include "agent.h"
#include "collision_avoidance.h"
#include "swarm_msgs/agentType.h"
#include "swarm_msgs/agentState.h"
#include "swarm_msgs/worldState.h"
#include "swarm_msgs/agentCommand.h"
#include "swarm_local_planner/CollisionAvoidance.h"

ros::Publisher command_pub;
ros::ServiceClient client;

struct MotionGoal{
    double x;   
    double y;
    double heading_rad;

    swarm_tools::Point2D position() const{
        return swarm_tools::Point2D{this->x, this->y};
    }
};

bool move_to_motion_goal(
    const agent::AgentState& agent_state,
    const agent::AgentConstraints& agent_constraints,
    const MotionGoal& motion_goal,
    agent::AgentCommand& command
);
bool is_in_interval(
    const double angle,
    const double lower_bound,
    const double upper_bound
){
    return lower_bound<= angle && angle<= upper_bound;
}

bool move_to_motion_goal(
    const agent::AgentState& agent_state,
    const agent::AgentConstraints& agent_constraints,
    const MotionGoal& motion_goal,
    agent::AgentCommand& command
){
    double offset_angle = -swarm_tools::absolute_angle_between_points(agent_state.position(),
                                                                      motion_goal.position());
    double heading = agent_state.heading;
    if (heading>swarm_tools::PI){
        heading -= 2*swarm_tools::PI;
    }
    double delta_heading;
    if (std::abs(offset_angle-agent_state.heading)<std::abs(-offset_angle+agent_state.heading))
        {delta_heading = offset_angle-agent_state.heading;}
    else
        {delta_heading = -offset_angle+agent_state.heading;}
    
    if (delta_heading>swarm_tools::PI){
        delta_heading -= 2*swarm_tools::PI;
        delta_heading *= -1;
    }else if(delta_heading<-swarm_tools::PI){
        delta_heading += 2*swarm_tools::PI;
        delta_heading *= -1;
    }

    delta_heading = swarm_tools::clip(delta_heading,
                                      -agent_constraints.max_delta_heading,
                                      agent_constraints.max_delta_heading);
    double delta_speed = agent_constraints.max_speed-agent_state.speed;
    delta_speed = swarm_tools::clip(delta_speed,
                                    -agent_constraints.max_delta_speed,
                                    agent_constraints.max_delta_speed);
    command.delta_speed=delta_speed;
    command.delta_heading=delta_heading;
    return true;
}

void callback(const swarm_msgs::worldState::ConstPtr& world_state){
    auto ws = world_state->worldState;

    std::vector<agent::AgentState> intruders;
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

            agent::AgentState usv = {x, y, speed, heading, radius, sim_id}; 
            double max_speed = 30;
            double max_delta_speed = 2; 
            double max_delta_heading =  swarm_tools::PI/9;
            agent::AgentConstraints constraints;
            constraints.max_speed = max_speed;
            constraints.max_delta_speed = max_delta_speed;
            constraints.max_delta_heading = max_delta_heading;
            agent::AgentCommand command;
            
            MotionGoal motion_goal;
            if (x<-200){
                motion_goal.x = 200;
                motion_goal.y = 0;
                ROS_INFO("HERE 0");
            }
            else if (x>200){
                motion_goal.x = -200;
                motion_goal.y = 0;
                ROS_INFO("HERE 1");
            }
            else if ((is_in_interval(heading, 0, swarm_tools::PI/2) || is_in_interval(heading, 1.5*swarm_tools::PI, 2*swarm_tools::PI))){
                motion_goal.x = 200;
                motion_goal.y = 0;
                ROS_INFO("HERE 2");
            }
            else{
                ROS_INFO("HERE 3");
                motion_goal.x = 200;
                motion_goal.y = 0;
            }
            ROS_INFO("Motion Goal ([%f], [%f])", motion_goal.x, motion_goal.y);

            move_to_motion_goal(usv,
                                constraints,
                                motion_goal,
                                command);

            double max_distance = 200;
            double max_angle_rad = swarm_tools::PI/4;
            double aggression = 0.5;
            
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
}

int main(int argc, char **argv){

    ros::init(argc, argv, "planner");

    ros::NodeHandle n;
    command_pub = n.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    ros::Subscriber sub = n.subscribe("Perception", 1000, callback);
    client = n.serviceClient<swarm_local_planner::CollisionAvoidance>("collision_avoidance");
    ros::spin();
    return 0;
    //ros::Rate loop_rate(10);
    // loop_rate.sleep();
}