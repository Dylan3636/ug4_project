#include <vector>
#include "swarm_tools.h"
#include "usv_swarm.h"

#ifndef MG_H
#define MG_H
namespace swarm_control{
    bool usv_delay_motion_goal(
        int usv_id,
        int intruder_id,
        const agent::USVSwarm& swarm);

   bool usv_block_motion_goal(
        const agent::AgentState& usv_state,
        const agent::AgentConstraints& usv_constraints,
        const double &capture_radius,
        const agent::AgentState& intruder_state,
        const agent::AgentConstraints& intruder_constraints,
        const double &intruder_radius,
        const agent::AgentState& asset_state,
        agent::MotionGoal& motion_goal);

    double get_smallest_delta_heading(double heading_goal,
                                      double current_heading,
                                      double max_delta_heading);

    template<typename T> bool get_batch_intruder_commands_from_model(
            const std::vector<T> &intruders,
            std::map<int, agent::AgentCommand> &commands,
            ros::ServiceClient &client);

    bool get_intruder_command_from_motion_goal(
            const agent::IntruderAgent& intruder,
            const agent::MotionGoal& motion_goal,
            agent::AgentCommand& command);

    bool get_observed_intruder_command_from_motion_goal(
            const agent::ObservedIntruderAgent& intruder,
            const agent::MotionGoal& motion_goal,
            agent::AgentCommand& command);

    bool get_usv_command_from_motion_goal(
            int usv_id,
            const agent::USVSwarm& swarm,
            const agent::MotionGoal& motion_goal,
            agent::AgentCommand &command);

    bool get_command_from_motion_goal(
        const agent::AgentState& agent_state,
        const agent::AgentConstraints& agent_constraints,
        const agent::MotionGoal& motion_goal,
        agent::AgentCommand& command);

    double weighted_motion_goal(
        const std::vector<agent::MotionGoal>& motion_goals,
        const std::vector<double>& weights,
        agent::MotionGoal& weighted_motion_goal);

    bool usv_guard_motion_goal(int num_of_usvs,
                               int guard_id,
                               double asset_radius,
                               const agent::AgentState asset_state,
                               agent::MotionGoal& motion_goal);

    bool usv_observe_motion_goal(int usv_id,
                                 int intruder_id,
                                 const agent::USVSwarm &swarm,
                                 agent::MotionGoal &motion_goal);

    double get_motion_goal_from_assignment(int usv_id,
                                         const agent::USVSwarm &swarm,
                                         agent::MotionGoal &motion_goal);

    bool intruder_motion_goal(const agent::AgentState &asset_state,
                              agent::MotionGoal &motion_goal);

    bool observed_intruder_motion_goal(
            int intruder_id,
            const agent::USVSwarm &swarm,
            agent::MotionGoal &motion_goal);
}
#endif