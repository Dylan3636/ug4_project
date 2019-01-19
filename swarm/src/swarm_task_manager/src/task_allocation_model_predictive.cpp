#include "task_allocation.h"
// #include "usv_swarm.h"
#include "agent_control.h"
#include <map>
#include <assert.h>
#include "ros/ros.h"

namespace swarm_task_manager{
    void share_assignment(agent::AgentAssignment &main_usv_assignment,
                          agent::AgentAssignment &other_usv_assignment){
        if (other_usv_assignment.delay_assignment_idx == -1){
            other_usv_assignment.delay_assignment_idx = main_usv_assignment.delay_assignment_idx;
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
        agent::AgentAssignment main_usv_assignment = swarm_assignment.at(sim_id);

        agent::SwarmAssignment swarm_assignment_copy;
        agent::AgentAssignment main_usv_assignment_copy; 
        agent::AgentAssignment other_usv_assignment_copy;

        std::vector<agent::WeightedSwarmAssignment> candidates={agent::WeightedSwarmAssignment(swarm_assignment, 0.0)};
        for (auto const &assignment_pair : swarm_assignment){
            if (sim_id==assignment_pair.first){
                std::cout << sim_id << std::endl;
                continue;
            }

            swarm_assignment_copy = swarm_assignment;
            main_usv_assignment_copy = main_usv_assignment;
            other_usv_assignment_copy = assignment_pair.second;

            share_assignment(main_usv_assignment_copy, other_usv_assignment_copy);

            swarm_assignment_copy[sim_id]=main_usv_assignment_copy;
            swarm_assignment_copy[assignment_pair.first] = other_usv_assignment_copy;
            std::cout << "MAIN USV : " << sim_id <<"GUARD ASSIGNMENT: " << main_usv_assignment_copy.guard_assignment_idx << "DELAY ASSINGMENT : " << main_usv_assignment_copy.delay_assignment_idx << std::endl;
            std::cout << "OTHER USV : " << assignment_pair.first <<"GUARD ASSIGNMENT: " << other_usv_assignment_copy.guard_assignment_idx << "DELAY ASSINGMENT : " << other_usv_assignment_copy.delay_assignment_idx << std::endl;

            candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));

            swarm_assignment_copy = swarm_assignment;
            main_usv_assignment_copy = main_usv_assignment;
            other_usv_assignment_copy = assignment_pair.second;

            exchange_assignment(&main_usv_assignment_copy.delay_assignment_idx,
                                &other_usv_assignment_copy.delay_assignment_idx);
            swarm_assignment_copy[sim_id]=main_usv_assignment_copy;
            swarm_assignment_copy[assignment_pair.first] = other_usv_assignment_copy;

            candidates.push_back(agent::WeightedSwarmAssignment(swarm_assignment_copy, 0.0));

            swarm_assignment_copy = swarm_assignment;
            main_usv_assignment_copy = main_usv_assignment;
            other_usv_assignment_copy = assignment_pair.second;

            exchange_assignment(&main_usv_assignment_copy.guard_assignment_idx,
                                &other_usv_assignment_copy.guard_assignment_idx);

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
        for (const auto &assignment : swarm_copy.get_swarm_assignment()){
            ROS_INFO("USV: %d DELAY ASSIGNMENT %d: GUARD ASSIGNMENT %d", assignment.first, assignment.second.delay_assignment_idx, assignment.second.guard_assignment_idx);
        }
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
                                                              guard_motion_goal,
                                                              delay_motion_goal,
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