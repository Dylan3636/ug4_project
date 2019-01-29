#include "agent_control.h"
namespace swarm_control{
    void get_next_usv_command_by_id(int usv_id,
                                   const agent::USVSwarm &swarm,
                                   agent::MotionGoal &motion_goal,
                                   agent::AgentCommand &command){

        int num_usvs = swarm.get_num_usvs();
        agent::USVAgent usv = swarm.get_usv_estimate_by_id(usv_id);
        agent::AssetAgent asset = swarm.get_asset_estimate();

        get_motion_goal_from_assignment(usv_id,
                                        swarm,
                                        motion_goal);

        get_command_from_motion_goal(usv.get_state(),
                                     usv.get_constraints(),
                                     motion_goal,
                                     command);

        collision_avoidance::correct_command(usv,
                                             swarm.get_obstacle_states(),
                                             command);
    }

    void get_next_intruder_command_by_id(int intruder_id,
                                        const agent::USVSwarm &swarm,
                                        agent::MotionGoal &intruder_mg,
                                        agent::AgentCommand &command){
    
        agent::IntruderAgent intruder = swarm.get_intruder_estimate_by_id(intruder_id);
        intruder_motion_goal(swarm.get_asset_estimate().get_state(),
                             intruder_mg);

        get_command_from_motion_goal(intruder.get_state(),
                                     intruder.get_constraints(),
                                     intruder_mg,
                                     command);

        collision_avoidance::correct_command(intruder,
                                             swarm.get_obstacle_states(),
                                             command);
    }
}