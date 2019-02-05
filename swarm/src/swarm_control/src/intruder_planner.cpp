#include <map>
#include <vector>
#include "agent.h"
#include "collision_avoidance.h"
#include "motion_goal_control.h"
#include "ros_swarm_tools.h"
#include "swarm_control/CollisionAvoidance.h"

ros::Publisher command_pub;
ros::ServiceClient client;
RosContainerPtr ros_container_ptr;
std::map<int, agent::IntruderAgent> intruder_map;
std::map<int, agent::CollisionAvoidanceParameters> intruder_radar_params_map;
std::map<int, agent::AgentConstraints> intruder_constraints_map;
std::string intruder_head_str = "/swarm_simulation/intruder_params";

void callback(const swarm_msgs::worldState::ConstPtr& world_state){
    auto ws = world_state->worldState;

    std::vector<agent::AgentState> intruders;
    agent::AgentState tanker;
    for ( const swarm_msgs::agentState &agent_state : ws){
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
                bool threat=false;
                auto mgs = intruder.get_motion_goals_ref();
                bool successful=get_intruder_motion_goals(
                        ros_container_ptr,
                        intruder_head_str,
                        sim_id, threat,
                        mgs);

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

    for (const auto &intruder_pair : intruder_map){

        int sim_id = intruder_pair.first;
        auto intruder = intruder_pair.second;

        agent::AgentCommand command{};
        agent::MotionGoal motion_goal;
        bool end = !intruder.get_motion_goal(&motion_goal);
        ROS_ASSERT(!end);
        
        swarm_control::get_intruder_command_from_motion_goal(intruder,
                                                             motion_goal,
                                                             command);


        swarm_control::CollisionAvoidance srv;
        
        // WorldState
        srv.request.world_state = *world_state;
        
        // Command
        srv.request.desired_command.sim_id=sim_id;
        srv.request.desired_command.delta_speed=command.delta_speed;
        srv.request.desired_command.delta_heading=command.delta_heading;
        
        // Constraints
        srv.request.agent_constraints.sim_id=sim_id;
        srv.request.agent_constraints.max_speed=intruder.get_max_speed();
        srv.request.agent_constraints.max_delta_speed=intruder.get_max_delta_speed();
        srv.request.agent_constraints.max_delta_heading=intruder.get_max_delta_heading();

        // Parameters
        srv.request.agent_params.sim_id=sim_id;
        srv.request.agent_params.max_distance=intruder.get_max_radar_distance();
        srv.request.agent_params.max_angle=intruder.get_max_radar_angle_rad();
        srv.request.agent_params.aggression=intruder.get_aggression();

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

    // Load Parameters
    get_agent_parameters(ros_container_ptr,
            intruder_head_str,
            intruder_constraints_map,
            intruder_radar_params_map);

    ros::spin();
    return 0;
}