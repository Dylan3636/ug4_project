#include "usv_swarm.h"
#include "collision_avoidance.h"
#include "motion_goal_control.h"

#ifndef SWARM_CONTROL_AGENT_CONTROL_H
#define SWARM_CONTROL_AGENT_CONTROL_H
namespace swarm_control{
    void get_next_usv_command_by_id(int usv_id,
                                   const agent::USVSwarm &swarm,
                                   agent::MotionGoal &motion_goal,
                                   agent::AgentCommand &command);

    void get_next_intruder_command_by_id(int intruder_id,
                                        const agent::USVSwarm &swarm,
                                        agent::MotionGoal &intruder_motion_goal,
                                        agent::AgentCommand &command);
    void get_batch_intruder_commands(const agent::USVSwarm &swarm,
            std::map<int, agent::AgentCommand> &intruder_commands_map);
}
#endif