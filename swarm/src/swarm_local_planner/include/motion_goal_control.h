#include <vector>
#include "agent.h"
#include "swarm_tools.h"

#ifndef MG_H
#define MG_H
namespace swarm_control{
    bool usv_delay_motion_goal(const agent::USVAgent &usv,
                               const agent::IntruderAgent &intruder,
                               const agent::AssetAgent &asset,
                               agent::MotionGoal motion_goal);

    bool usv_delay_motion_goal(
        const agent::AgentState& usv_state,
        const agent::AgentConstraints& usv_constraints,
        const agent::AgentState& intruder_state,
        const agent::AgentConstraints& intruder_constraints,
        const agent::AgentState& asset_state,
        agent::MotionGoal& motion_goal
    );
    bool usv_block_motion_goal(
        const agent::AgentState& usv_state,
        const agent::AgentConstraints& usv_constraints,
        const double &capture_radius,
        const agent::AgentState& intruder_state,
        const agent::AgentConstraints& intruder_constraints,
        const double &intruder_radius,
        const agent::AgentState& asset_state,
        agent::MotionGoal& motion_goal
    );

    bool move_to_motion_goal(
        const agent::AgentState& agent_state,
        const agent::AgentConstraints& agent_constraints,
        const agent::MotionGoal& motion_goal,
        agent::AgentCommand& command
    );

    bool weighted_motion_goal(
        const std::vector<agent::MotionGoal>& motion_goals,
        const std::vector<double>& weights,
        agent::MotionGoal& weighted_motion_goal
    );
    bool usv_guard_motion_goal(const int& num_of_usvs,
                               const int& usv_assignment,
                               const double& radius,
                               const agent::AgentState asset_state,
                               agent::MotionGoal& motion_goal
    );
}
#endif