#include "task_allocation.h"
// #include "usv_swarm.h"
#include "agent_control.h"
#include <map>
#include <assert.h>

namespace swarm_task_manager{
    void share_assignment(agent::AgentAssignment &main_usv_assignment,
                          agent::AgentAssignment &other_usv_assignment){
        if (other_usv_assignment.delay_assignment_idx != -1){
            int temp_assignment = main_usv_assignment.delay_assignment_idx;
            main_usv_assignment.delay_assignment_idx = other_usv_assignment.delay_assignment_idx;
            other_usv_assignment.delay_assignment_idx = temp_assignment;
        }
        other_usv_assignment.delay_assignment_idx = main_usv_assignment.delay_assignment_idx;
    }
    void exchange_assignment(agent::AgentAssignment &main_usv_assignment,
                             agent::AgentAssignment &other_usv_assignment){
        
        int temp_assignment = main_usv_assignment.guard_assignment_idx;
        main_usv_assignment.guard_assignment_idx = other_usv_assignment.guard_assignment_idx;
        other_usv_assignment.guard_assignment_idx = temp_assignment;
    }

    std::vector<agent::WeightedSwarmAssignment> generate_assignment_candidates(int sim_id,
                                                                               const agent::SwarmAssignment &swarm_assignment,
                                                                               const std::map<int, bool> &communication_map){

        assert(swarm_assignment.find(sim_id)!=swarm_assignment.end());
        agent::AgentAssignment main_usv_assignment = swarm_assignment.at(sim_id);

        agent::SwarmAssignment swarm_assignment_copy;
        agent::AgentAssignment main_usv_assignment_copy; 
        agent::AgentAssignment other_usv_assignment_copy;

        std::vector<agent::WeightedSwarmAssignment> candidates={agent::WeightedSwarmAssignment(swarm_assignment, 0.0)};
        for (auto const &assignment_pair : swarm_assignment){
            if (sim_id==assignment_pair.first){
                continue;
            }

            swarm_assignment_copy = swarm_assignment;
            main_usv_assignment_copy = main_usv_assignment;
            other_usv_assignment_copy = assignment_pair.second;

            share_assignment(main_usv_assignment_copy, other_usv_assignment_copy);
            swarm_assignment_copy[sim_id]=main_usv_assignment_copy;
            swarm_assignment_copy[assignment_pair.first] = other_usv_assignment_copy;

            candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));

            swarm_assignment_copy = swarm_assignment;
            main_usv_assignment_copy = main_usv_assignment;
            other_usv_assignment_copy = assignment_pair.second;

            exchange_assignment(main_usv_assignment_copy, other_usv_assignment_copy);

            swarm_assignment_copy[sim_id]=main_usv_assignment_copy;
            swarm_assignment_copy[assignment_pair.first] = other_usv_assignment_copy;
            candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));
        }
    return candidates;
    }

    double evaluate_swarm_assignment(const agent::SwarmAssignment &swarm_assignment,
                                     const agent::USVSwarm &swarm,
                                     int num_timesteps,
                                     double delta_time_secs,
                                     double threshold){

        int sim_id;
        agent::AgentType agent_type;
        agent::USVSwarm swarm_copy = swarm;
        auto agent_sim_id_map = swarm.get_agent_sim_id_map();
        agent::AgentCommand command;
        agent::MotionGoal usv_motion_goal;
        agent::MotionGoal delay_motion_goal;
        agent::MotionGoal guard_motion_goal;
        agent::MotionGoal intruder_motion_goal;

        int timestep=0;

        while(timestep < num_timesteps){
            for (const auto &sim_id_type_pair : agent_sim_id_map){
                sim_id = sim_id_type_pair.first;
                agent_type = sim_id_type_pair.second;
                if (agent_type == agent::USV){ 
                    swarm_control::get_next_usv_command_by_id(sim_id,
                                                              swarm,
                                                              usv_motion_goal,
                                                              guard_motion_goal,
                                                              delay_motion_goal,
                                                              command);
                    swarm_copy.command_usv_forward_by_id(sim_id,
                                                         command,
                                                         delta_time_secs);
                }
                else if (agent_type == agent::Intruder){
                    swarm_control::get_next_intruder_command_by_id(sim_id,
                                                                   swarm,
                                                                   intruder_motion_goal,
                                                                   command);
                    swarm_copy.command_intruder_forward_by_id(sim_id,
                                                              command,
                                                              delta_time_secs);
                    agent::IntruderAgent intruder = swarm_copy.get_intruder_estimate_by_id(sim_id_type_pair.first);
                    double dist_to_asset = swarm_tools::euclidean_distance(intruder.get_state().get_position(),
                                                                           swarm.get_asset_estimate().get_position());
                    if(dist_to_asset<threshold) return timestep;
                }
            }
            timestep++;
        }
        return timestep;
    }
    void weight_candidate_swarm_assignments(const agent::USVSwarm swarm,
                                              int num_timesteps,
                                              double delta_time_secs,
                                              double threshold,
                                              std::vector<agent::WeightedSwarmAssignment> &candidate_assignments){
        double weight;
        for (auto &assignment_pair : candidate_assignments){
            weight = evaluate_swarm_assignment(assignment_pair.first,
                                               swarm,
                                               num_timesteps,
                                               delta_time_secs,
                                               threshold);
            assignment_pair.second=weight;
        }
    }

    std::vector<agent::WeightedSwarmAssignment> generate_weighted_candidate_assignments(int sim_id,
                                                                                 const agent::USVSwarm &swarm,
                                                                                 const std::map<int, bool> &communication_map,
                                                                                 int num_timesteps,
                                                                                 double delta_time_secs,
                                                                                 double threshold){
        auto candidate_assignments = generate_assignment_candidates(sim_id, swarm.get_swarm_assignment(), communication_map);
        weight_candidate_swarm_assignments(swarm,
                                           num_timesteps,
                                           delta_time_secs,
                                           threshold,
                                           candidate_assignments);
        return candidate_assignments;
    }
    agent::WeightedSwarmAssignment max_weighted_swarm_assignment(
        const std::vector<agent::WeightedSwarmAssignment> &weighted_assignments){
        assert(weighted_assignments.size()!=0);
        agent::WeightedSwarmAssignment max_assignment=weighted_assignments[0];
        for (const auto &assignment_pair : weighted_assignments){
            if(max_assignment.second<assignment_pair.second){
                max_assignment=assignment_pair;
            }
        }
        return max_assignment;
    }
    agent::WeightedSwarmAssignment get_best_candidate_swarm_assignment(int sim_id,
                                                                       const agent::USVSwarm &swarm,
                                                                       const std::map<int, bool> communication_map,
                                                                       int num_timesteps,
                                                                       double delta_time_secs,
                                                                       double threshold){

        auto weighted_assignments = generate_weighted_candidate_assignments(sim_id, swarm, communication_map, num_timesteps, delta_time_secs, threshold);
        auto max_assignment = max_weighted_swarm_assignment(weighted_assignments);
        return max_assignment;
    }
}