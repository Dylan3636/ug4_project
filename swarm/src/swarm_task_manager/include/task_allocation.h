#include "usv_swarm.h"

#ifndef SWARM_TASK_MANAGER_TASK_ALLOCATION_H
#define SWARM_TASK_MANAGER_TASK_ALLOCATION_H
namespace swarm_task_manager{
   void generate_weighted_candidate_assignments(int sim_id,
                                                const agent::USVSwarm &swarm,
                                                const std::map<int, bool> &communication_map,
                                                int num_timesteps,
                                                double delta_time_secs,
                                                double threshold,
                                                std::vector<agent::WeightedSwarmAssignment> &wsa);

    agent::WeightedSwarmAssignment get_best_candidate_swarm_assignment(int sim_id,
                                                                const agent::USVSwarm &swarm,
                                                                const std::map<int, bool> communication_map,
                                                                int num_timesteps,
                                                                double delta_time_secs,
                                                                double threshold);

    agent::WeightedSwarmAssignment max_weighted_swarm_assignment(
       const std::vector<agent::WeightedSwarmAssignment> &weighted_assignments);
}
#endif