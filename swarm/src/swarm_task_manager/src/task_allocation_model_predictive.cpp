#include "task_allocation.h"
// #include "usv_swarm.h"
#include "agent_control.h"
#include <map>
#include <assert.h>
#include "ros/ros.h"

namespace swarm_task_manager{
    
    bool share_assignment(int *main_usv_assignment_index,
                          int *other_usv_assignment_index){
        if(*other_usv_assignment_index == -1){
            *other_usv_assignment_index = *main_usv_assignment_index;        
            return true;
        }else{
            return false;
        }
    }
    void exchange_assignment(int *main_usv_assignment_index,
                             int *other_usv_assignment_index){
        
        int temp_assignment = *main_usv_assignment_index;
        if(*main_usv_assignment_index == *other_usv_assignment_index){
            *main_usv_assignment_index = -1;
        }else{
            *main_usv_assignment_index = *other_usv_assignment_index;
            *other_usv_assignment_index = temp_assignment;
        }
    }

    std::vector<agent::WeightedSwarmAssignment> generate_assignment_candidates(int sim_id,
                                                                               const agent::SwarmAssignment &swarm_assignment,
                                                                               const std::map<int, bool> &communication_map){

        assert(swarm_assignment.find(sim_id)!=swarm_assignment.end());

        agent::SwarmAssignment swarm_assignment_copy = swarm_assignment;
        agent::AgentAssignment &main_usv_assignment = swarm_assignment_copy.at(sim_id); 
        // agent::AgentAssignment &other_usv_assignment;
        std::vector<agent::WeightedSwarmAssignment> candidates={agent::WeightedSwarmAssignment(swarm_assignment, 0.0)};

        std::set<int> possible_duplicates;
        bool has_delay_task;
        bool has_delay=false;
        int main_usv_task_idx_tmp;
        int other_usv_task_idx_tmp;

        for (auto &assignment_pair : swarm_assignment_copy){
            if (sim_id==assignment_pair.first){
                std::cout << sim_id << std::endl;
                continue;
            }
            has_delay_task=false;
            agent::AgentAssignment &other_usv_assignment = assignment_pair.second;
            for(const auto &other_usv_task : other_usv_assignment){
                if(other_usv_task.task_type==agent::TaskType::Observe){
                    possible_duplicates.emplace(other_usv_task.task_idx);
                }else if(other_usv_task.task_type==agent::TaskType::Delay){
                    has_delay_task=true;
                }
            }
            for(auto &main_usv_task : main_usv_assignment){
                for(auto &other_usv_task : other_usv_assignment){
                    if(main_usv_task.task_type==other_usv_task.task_type){
                        // Exchange Task

                        main_usv_task_idx_tmp = main_usv_task.task_idx;
                        other_usv_task_idx_tmp = other_usv_task.task_idx;
                        ROS_INFO("BEFORE EXCHANGE MAIN: %d OTHER %d", main_usv_task.task_idx, other_usv_task.task_idx);
                        exchange_assignment(&main_usv_task.task_idx, &other_usv_task.task_idx);
                        ROS_INFO("AFTER EXCHANGE MAIN: %d OTHER %d", main_usv_task.task_idx, other_usv_task.task_idx);
                        candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));
                        // reset
                        main_usv_task.task_idx = main_usv_task_idx_tmp;
                        other_usv_task.task_idx = other_usv_task_idx_tmp;
                        ROS_INFO("AFTER RESET MAIN: %d OTHER %d", main_usv_task.task_idx, other_usv_task.task_idx);

                        if(main_usv_task.task_type==agent::TaskType::Guard){
                            continue;
                        }else{
                            main_usv_task_idx_tmp = main_usv_task.task_idx;
                            other_usv_task_idx_tmp = other_usv_task.task_idx;

                            ROS_INFO("BEFORE SHARE MAIN: %d OTHER %d", main_usv_task.task_idx, other_usv_task.task_idx);
                            bool successful = share_assignment(&main_usv_task.task_idx, &other_usv_task.task_idx);
                            if(successful){
                                ROS_INFO("AFTER SHARE MAIN: %d OTHER %d", main_usv_task.task_idx, other_usv_task.task_idx);
                                candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));
                            }else{
                                if(main_usv_task.task_idx==other_usv_task.task_idx){
                                    ROS_INFO("REMOVING %d TASK", main_usv_task.task_type);
                                    main_usv_task.task_idx=-1;
                                    candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));
                                }
                            }

                            // reset
                            main_usv_task.task_idx = main_usv_task_idx_tmp;
                            other_usv_task.task_idx = other_usv_task_idx_tmp;
                            ROS_INFO("AFTER RESET MAIN: %d OTHER %d", main_usv_task.task_idx, other_usv_task.task_idx);
                        }
                   }else{
                        // Share task
                        if(main_usv_task.task_type==agent::TaskType::Observe){
                            if(possible_duplicates.find(main_usv_task.task_idx) != possible_duplicates.end()){
                                ROS_INFO("SHARING OBSERVE TASK MAIN %d", main_usv_task.task_idx);
                                other_usv_assignment.push_back(main_usv_task);
                                candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));
                                other_usv_assignment.pop_back(); // reset
                            }
                        }
                        else if(main_usv_task.task_type==agent::TaskType::Delay){
                            if(!has_delay_task){
                                ROS_INFO("SHARING DELAY TASK MAIN %d OTHER %d", main_usv_task.task_idx);
                                other_usv_assignment.push_back(main_usv_task);
                                candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));
                                other_usv_assignment.pop_back(); // reset
                            }
                        }
                   }
                }
            }
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
        swarm_copy.update_swarm_assignment(swarm_assignment);
        auto agent_sim_id_map = swarm.get_agent_sim_id_map();
        agent::AgentCommand command;
        agent::MotionGoal usv_motion_goal;
        agent::MotionGoal delay_motion_goal;
        agent::MotionGoal guard_motion_goal;
        agent::MotionGoal intruder_motion_goal;

        int timestep=0;
        double min_dist_to_asset=-1;
        double dist_to_asset=-1;
        double weight=0;
        ROS_INFO("Evaluating swarm assignment:");
        ROS_INFO(agent::swarm_assignment_to_string(swarm_copy.get_swarm_assignment()).c_str());
        bool terminated=false;
        while(!terminated){
            min_dist_to_asset=1000000;
            for (const auto &sim_id_type_pair : agent_sim_id_map){
                sim_id = sim_id_type_pair.first;
                agent_type = sim_id_type_pair.second;
                if (agent_type == agent::USV){ 
                    swarm_control::get_next_usv_command_by_id(sim_id,
                                                              swarm_copy,
                                                              usv_motion_goal,
                                                              command);
                    // ROS_INFO("USV %d COMMAND (%f, %f)", sim_id, command.delta_heading, command.delta_speed);
                    swarm_copy.command_usv_forward_by_id(sim_id,
                                                         command,
                                                         delta_time_secs);
                }
                else if (agent_type == agent::Intruder){
                    swarm_control::get_next_intruder_command_by_id(sim_id,
                                                                   swarm_copy,
                                                                   intruder_motion_goal,
                                                                   command);
                    // ROS_INFO("INTRUDER %d COMMAND (%f, %f)", sim_id, command.delta_heading, command.delta_speed);
                    swarm_copy.command_intruder_forward_by_id(sim_id,
                                                              command,
                                                              delta_time_secs);
                    agent::IntruderAgent intruder = swarm_copy.get_intruder_estimate_by_id(sim_id_type_pair.first);
                    dist_to_asset = swarm_tools::euclidean_distance(intruder.get_state().get_position(),
                                                                           swarm_copy.get_asset_estimate().get_position());
                    if (min_dist_to_asset>dist_to_asset){
                        min_dist_to_asset = dist_to_asset;
                        weight = min_dist_to_asset/intruder.get_speed();
                        //ROS_INFO("SPEED %f, DIST %f, WEIGHT %f", intruder.get_speed(), min_dist_to_asset, weight);
                    }
                    // ROS_INFO("Intruder %d STATE \nx=%f\ny=%f\nspeed=%f\nheading=%f",
                    //                     sim_id,
                    //                     intruder.get_x(),
                    //                     intruder.get_y(),
                    //                     intruder.get_speed(),
                    //                     intruder.get_heading());

                    if(min_dist_to_asset<threshold){
                        terminated=true;
                        ROS_INFO("DISTANCE TO ASSET: %f", min_dist_to_asset);
                        break;};
                }
            }
            if(terminated || timestep>num_timesteps){break;}
            timestep++;
        }
        ROS_INFO("WEIGHT: %f", weight);
        ROS_INFO("DISTANCE TO ASSET: %f", min_dist_to_asset);
        return weight;
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