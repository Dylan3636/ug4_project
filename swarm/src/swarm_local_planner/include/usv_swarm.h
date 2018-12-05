#include <map>
#include <vector>
#include "agent.h"

#ifndef USV_SWARM_H
#define USV_SWARM_H

namespace agent{
    typedef std::map<int, AgentAssignment> SwarmAssignment;

    class USVSwarm{
        std::map<int, USVAgent> usv_map;
        std::map<int, IntruderAgent> intruder_map;
        AssetAgent asset;

        // private methods
        void update_intruder_state_estimate(const AgentState &intruder_state);
        void update_intruder_estimate(const IntruderAgent &intruder);
        void update_usv_state_estimate(const AgentState &usv_state);
        void update_usv_estimate(const USVAgent &usv);

        public:
            int get_num_usvs() const;
            IntruderAgent get_intruder_estimate_by_id(int intruder_id) const;
            USVAgent get_usv_estimate_by_id(int usv_id) const;
            AssetAgent get_asset_estimate() const;
            std::map<int, AgentType> USVSwarm::get_agent_sim_id_map() const;

            std::vector<USVAgent> get_usv_estimates() const;
            std::vector<IntruderAgent> get_intruder_estimates() const;
            std::vector<AgentState> get_obstacle_states() const;
            SwarmAssignment get_swarm_assignment() const;

            void command_usv_forward_by_id(int usv_id,
                                           const AgentCommand &command,
                                           double delta_time_secs);
            void command_intruder_forward_by_id(int intruder_id,
                                                          const AgentCommand &command,
                                                          double delta_time_secs);
            int get_next_intruder_command_by_id(int intruder_id,
                                                MotionGoal &intruder_motion_goal,
                                                AgentCommand &command) const;
            int get_next_usv_command_by_id(int usv_id,
                                           MotionGoal &motion_goal,
                                           MotionGoal &guard_motion_goal,
                                           MotionGoal &delay_motion_goal,
                                           AgentCommand &command) const;

            void update_estimates(const std::map<int, AgentState> &usv_state_map,
                                  const std::map<int, AgentState> &intruder_state_map);
            void update_estimates(const std::map<int,
                                      agent::USVAgent> &new_usv_map,
                                  const std::map<int,
                                      agent::IntruderAgent> &new_intruder_map);

            std::vector<double> evaluate_swarm_assignment(const SwarmAssignment &swarm_assignment,
                                                          int timesteps,
                                                          double delta_time_secs);
    };
    int get_next_usv_command_by_id(int usv_id,
                                   const USVSwarm &swarm,
                                   agent::MotionGoal &motion_goal,
                                   agent::MotionGoal &guard_motion_goal,
                                   agent::MotionGoal &delay_motion_goal,
                                   agent::AgentCommand &command);
}

#endif