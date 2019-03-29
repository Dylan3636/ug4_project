#include "agent_control.h"
#include "motion_goal_control.h"
namespace swarm_control{
    void get_next_usv_command_by_id(int usv_id,
                                   const agent::USVSwarm &swarm,
                                   agent::MotionGoal &motion_goal,
                                   agent::AgentCommand &command){

        agent::USVAgent usv = swarm.get_usv_estimate_by_id(usv_id);

        get_motion_goal_from_assignment(usv_id,
                                        swarm,
                                        motion_goal);

        double aggression = get_usv_command_from_motion_goal(usv_id,
                                         swarm,
                                         motion_goal,
                                         command);
        usv.set_aggression(aggression);
        std::vector<agent::AgentState> obstacle_states;
        swarm.get_obstacle_states(obstacle_states);
        collision_avoidance::correct_command(usv,
                                             obstacle_states,
                                             command);
    }

    void get_next_intruder_command_by_id(int intruder_id,
                                         const agent::USVSwarm &swarm,
                                         agent::MotionGoal &intruder_mg,
                                         agent::AgentCommand &command){
    
        agent::ObservedIntruderAgent intruder = swarm.get_intruder_estimate_by_id(intruder_id);
        observed_intruder_motion_goal(
                intruder_id,
                swarm,
                intruder_mg);

        get_observed_intruder_command_from_motion_goal(swarm.get_intruder_estimate_by_id(intruder_id),
                                                       intruder_mg,
                                                       command);

        std::vector<agent::AgentState> obstacle_states;
        swarm.get_obstacle_states(obstacle_states);
        collision_avoidance::correct_command(intruder,
                obstacle_states,
                command);
    }
    void get_batch_intruder_commands(const agent::USVSwarm &swarm, std::map<int, agent::AgentCommand> &intruder_commands_map){
        std::vector<agent::ObservedIntruderAgent> intruders;
        swarm.get_intruder_estimates(intruders);
        ros::ServiceClient client=swarm.intruder_model_client;
        swarm_control::get_batch_intruder_commands_from_model(intruders, intruder_commands_map, client);
    }
}