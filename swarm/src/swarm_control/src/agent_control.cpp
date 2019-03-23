#include "agent_control.h"
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
        collision_avoidance::correct_command(usv,
                                             swarm.get_obstacle_states(),
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

        collision_avoidance::correct_command(intruder,
                                             swarm.get_obstacle_states(),
                                             command);
    }
    void get_batch_intruder_command(const agent::USVSwarm &swarm){
        auto intruders = swarm.get_intruder_estimates();
    }
}