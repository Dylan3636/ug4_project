#include <map>
#include <vector>
#include "agent.h"

#ifndef USV_SWARM_H
#define USV_SWARM_H

namespace agent{
    class USVSwarm{
        int main_usv_id;
        std::map<int, USVAgent> usv_map;
        std::map<int, ObservedIntruderAgent> intruder_map;
        ros::ServiceClient threat_detection_client;
        AssetAgent asset;

        // private  methods
        void update_intruder_estimate(const ObservedIntruderAgent &intruder);
        void update_usv_estimate(const USVAgent &usv);

        public:
            bool block_next_task_allocation=false;
            void reset();
            bool switch_observe_to_delay_task(int intruder_id);

            // Contains
            bool contains_usv(int usv_id);
            bool contains_intruder(int intruder_id);

            /* Getters*/
            // USV
            int get_main_usv_id(){
                return main_usv_id;
            }
            int get_num_usvs() const;
            USVAgent get_usv_estimate_by_id(int usv_id) const;
            std::vector<USVAgent> get_usv_estimates() const;
            std::vector<int> get_usv_ids() const;
            int get_delay_position(int usv_id, int intruder_id) const;
            AgentAssignment get_assignment_by_id(int usv_id) const;

            // Intruders
            ObservedIntruderAgent get_intruder_estimate_by_id(int intruder_id) const;
            std::vector<ObservedIntruderAgent> get_intruder_estimates() const;
            void sample_intruders();

            // Asset
            AssetAgent get_asset_estimate() const;

            // Agents
            std::map<int, AgentType> get_agent_sim_id_map() const;
            std::vector<AgentState> get_obstacle_states() const;
            SwarmAssignment get_swarm_assignment() const;


            // Setters
            void set_main_usv_id(int usv_id){
                main_usv_id=usv_id;
            }
            void set_threat_detection_client(const ros::ServiceClient &client){
                threat_detection_client=client;
            }

            // Command agents monitored by swarm
            void command_usv_forward_by_id(int usv_id,
                                           const AgentCommand &command,
                                           double delta_time_secs);
            void command_intruder_forward_by_id(int intruder_id,
                                                          const AgentCommand &command,
                                                          double delta_time_secs);

            // Adders
            void add_usv(const USVAgent &usv);
            void add_intruder(ObservedIntruderAgent intruder);

            // Update
            void update_intruder_state_estimate(const AgentState &intruder_state);

            void update_usv_state_estimate(const AgentState &usv_state);

            void update_swarm_assignment(const SwarmAssignment &swarm_assignment);

            void update_agent_assignment_by_id(int sim_id,
                                               const AgentAssignment &assignment);

            void update_estimates(const std::map<int, AgentState> &usv_state_map,
                                  const std::map<int, AgentState> &intruder_state_map);
            void update_estimates(const std::map<int,
                                      agent::USVAgent> &new_usv_map,
                                  const std::map<int,
                                      agent::ObservedIntruderAgent> &new_intruder_map);
            void update_usv_estimates(const std::map<int,
                                      agent::AgentState> &usv_state_map);
            void update_intruder_estimates(const std::map<int,
                                           agent::AgentState> &intruder_state_map);
            bool update_intruder_threat_estimate(const AgentState &intruder_state);

            // Assign
            void assign_intruder_to_usv(const ObservedIntruderAgent &intruder);
            void assign_guard_task_to_usv(int usv_id);

            // Sort
            std::vector<int> sort_usvs_by_distance_to_point(const swarm_tools::Point2D &point) const;
            std::vector<int> sort_usvs_by_distance_to_point(
                    const std::vector<USVAgent> &usv,
                    const swarm_tools::Point2D &point) const;

            std::vector<int> sort_usvs_by_weighted_distance_to_point(const swarm_tools::Point2D &point) const;
            std::vector<int> sort_usvs_by_weighted_distance_to_point(
                    const std::vector<USVAgent> &usvs,
                    const swarm_tools::Point2D &point) const;
    };


    int get_next_usv_command_by_id(int usv_id,
                                   const USVSwarm &swarm,
                                   agent::MotionGoal &motion_goal,
                                   agent::MotionGoal &guard_motion_goal,
                                   agent::MotionGoal &delay_motion_goal,
                                   agent::AgentCommand &command);
}

#endif