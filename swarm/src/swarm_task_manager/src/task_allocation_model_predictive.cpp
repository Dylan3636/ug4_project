#include "task_allocation.h"
#include "agent_control.h"
#include <map>
#include <vector>
#include <assert.h>
#include <thread>
#include <functional>
#include "ros/ros.h"
#include "ctime"
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
                ROS_INFO("Sim ID: %d", sim_id);
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
    void run_swarm_simulation(agent::USVSwarm swarm,
                              int num_timesteps,
                              double delta_time_secs,
                              double threshold,
                              double &weight) {
       int sim_id;
       agent::AgentType agent_type;
       agent::AgentCommand command;
       agent::MotionGoal usv_motion_goal;
       agent::MotionGoal intruder_motion_goal;
       if (swarm.get_usv_ids().empty()) ROS_ERROR("USV MAP EMPTY 1");
       if (swarm.get_intruder_estimates().empty()) ROS_ERROR("INTRUDER MAP EMPTY 1");
       auto agent_sim_id_map = swarm.get_agent_sim_id_map();
       if (agent_sim_id_map.empty()) ROS_ERROR("AGENT MAP EMPTY");
       if (swarm.get_usv_ids().empty()) ROS_ERROR("USV MAP EMPTY 2");
       if (swarm.get_intruder_estimates().empty()) ROS_ERROR("INTRUDER MAP EMPTY 2");

       int timestep=0;
       double min_dist_to_asset=-1;
       double dist_to_asset=-1;
       bool terminated=false;
       time_t sample_t= std::clock();
       // ROS_INFO("%s", agent::swarm_assignment_to_string(swarm.get_swarm_assignment()).c_str());
       while (!terminated) {
           // std::cout << "ID "<< std::this_thread::get_id();
           min_dist_to_asset = 1000000;
           time_t update_t= std::clock();
           for (const auto &sim_id_type_pair : agent_sim_id_map) {
               sim_id = sim_id_type_pair.first;
               agent_type = sim_id_type_pair.second;
               if (agent_type == agent::USV) {
                   swarm_control::get_next_usv_command_by_id(sim_id,
                                                             swarm,
                                                             usv_motion_goal,
                                                             command);
                   // ROS_INFO("USV %d COMMAND (%f, %f)", sim_id, command.delta_heading, command.delta_speed);
                   swarm.command_usv_forward_by_id(sim_id,
                                                   command,
                                                   delta_time_secs);
               } else if (agent_type == agent::Intruder) {
                   swarm_control::get_next_intruder_command_by_id(sim_id,
                                                                  swarm,
                                                                  intruder_motion_goal,
                                                                  command);
                   // ROS_INFO("INTRUDER %d COMMAND (%f, %f)", sim_id, command.delta_heading, command.delta_speed);
                   swarm.command_intruder_forward_by_id(sim_id,
                                                             command,
                                                             delta_time_secs);
                   agent::ObservedIntruderAgent intruder = swarm.get_intruder_estimate_by_id(
                           sim_id_type_pair.first);
                   dist_to_asset = swarm_tools::euclidean_distance(intruder.get_state().get_position(),
                                                                   swarm.get_asset_estimate().get_position());
                   if (min_dist_to_asset > dist_to_asset) {
                       min_dist_to_asset = dist_to_asset;
                       weight = min_dist_to_asset / intruder.get_speed();
                       //ROS_INFO("SPEED %f, DIST %f, WEIGHT %f", intruder.get_speed(), min_dist_to_asset, weight);
                   }
                   // ROS_INFO("Intruder %d STATE \nx=%f\ny=%f\nspeed=%f\nheading=%f",
                   //                     sim_id,
                   //                     intruder.get_x(),
                   //                     intruder.get_y(),
                   //                     intruder.get_speed(),
                   //                     intruder.get_heading());

                   if (min_dist_to_asset < threshold) {
                       terminated = true;
                       // ROS_INFO("DISTANCE TO ASSET: %f", min_dist_to_asset);
                       break;
                   };
               }
           }
           // ROS_INFO("Update time %f", (std::clock()-update_t)/(double) CLOCKS_PER_SEC);
           if (terminated || timestep > num_timesteps) { break; }
           timestep++;
       }
       ROS_INFO("Sample time %f", (std::clock()-sample_t)/(double) CLOCKS_PER_SEC);
    }

    void evaluate_swarm_assignment(const agent::SwarmAssignment &swarm_assignment,
                                   const agent::USVSwarm &swarm,
                                   int num_timesteps,
                                   double delta_time_secs,
                                   double threshold,
                                   double &weight){
        bool use_threads=true;
        int num_samples=50;
        try {
            ROS_INFO("Evaluating swarm assignment:");
            // ROS_INFO("%s", agent::swarm_assignment_to_string(swarm_assignment).c_str());
            std::thread threads[num_samples];
            double weights[num_samples];
            for (int i = 0; i < num_samples; i++) {
                // ROS_INFO("ORIGINAL USV MAP SIZE %d", swarm.get_usv_ids().size());
                agent::USVSwarm swarm_copy = swarm;
                // ROS_INFO("COPY USV MAP SIZE %d", swarm_copy.get_usv_ids().size());
                swarm_copy.update_swarm_assignment(swarm_assignment);
                swarm_copy.sample_intruders();
                if (use_threads) {
                    // ROS_INFO("Starting run_swarm_simulation thread");
                    threads[i] = std::thread(run_swarm_simulation,
                                             swarm_copy,
                                             num_timesteps,
                                             delta_time_secs,
                                             threshold,
                                             std::ref(weights[i]));
                    // ROS_INFO("After run_swarm_simulation thread");
                }else{
                    double sample_weight=0;
                    run_swarm_simulation(
                           swarm_copy,
                           num_timesteps,
                           delta_time_secs,
                           threshold,
                           sample_weight);
                    weights[i] = sample_weight;
                }
            }
            if (use_threads) {
                for (int j = 0; j < num_samples; j++) {
                    try {
                        // ROS_INFO("Waiting for thread to be joined");
                        while(!threads[j].joinable()) ROS_INFO("Thread not joinable");
                        threads[j].join();
                        // ROS_INFO("Thread joined");
                    } catch (std::exception &e) {
                        ROS_ERROR("Inner thread join error: %s", e.what());
                    }
                }
            }
            double total_weight=0.0;
            for(auto sam_weight : weights){
                total_weight += sam_weight;
            }
            weight = total_weight/(double) num_samples;
            // ROS_INFO("WEIGHT: %f", weight);
            // ROS_INFO("DISTANCE TO ASSET: %f", min_dist_to_asset);
        }catch(std::exception &e){
            ROS_ERROR("Error in thread %s", e.what());
        }
    }
    void weight_candidate_swarm_assignments(const agent::USVSwarm &swarm,
                                            int num_timesteps,
                                            double delta_time_secs,
                                            double threshold,
                                            std::vector<agent::WeightedSwarmAssignment> &candidate_assignments){
        std::thread threads[candidate_assignments.size()];
        int i = 0;
        bool use_threads=true;
        double weights[candidate_assignments.size()];
        for (auto &assignment_pair : candidate_assignments){
            // time_t t= std::clock();
            ROS_INFO("STARTING THREAD");
            if(use_threads) {
                threads[i] = std::thread(evaluate_swarm_assignment,
                                         std::ref(assignment_pair.first),
                                         std::ref(swarm),
                                         num_timesteps,
                                         delta_time_secs,
                                         threshold,
                                         std::ref(weights[i]));
                // ROS_INFO("Evaluation time %f", (std::clock()-t)/(double) CLOCKS_PER_SEC);
            }else{
                double weight=-100;
                evaluate_swarm_assignment(
                                         assignment_pair.first,
                                         swarm,
                                         num_timesteps,
                                         delta_time_secs,
                                         threshold,
                                         weight);
                assignment_pair.second = weight;
            }
            // thread.join();
            i++;
        }
        if (use_threads) {
            try {
                for (i = 0; i < candidate_assignments.size(); i++) {
                    try {
                        threads[i].join();
                    } catch (std::exception &e) {
                        ROS_ERROR("Thread join error: %s", e.what());
                    }
                }
                i=0;
                for (auto &assignment_pair : candidate_assignments) {
                    assignment_pair.second = weights[i];
                    i++;
                }
            }
            catch (std::exception &e) {
                ROS_ERROR("Thread loop error: %s", e.what());
            }
        }
    }

    std::vector<agent::WeightedSwarmAssignment> generate_weighted_candidate_assignments(int sim_id,
                                                                                 const agent::USVSwarm &swarm,
                                                                                 const std::map<int, bool> &communication_map,
                                                                                 int num_timesteps,
                                                                                 double delta_time_secs,
                                                                                 double threshold){

        time_t t= std::clock();
        auto candidate_assignments = generate_assignment_candidates(sim_id, swarm.get_swarm_assignment(), communication_map);
        ROS_INFO("Generation time %f", (std::clock()-t)/(double) CLOCKS_PER_SEC);
        t= std::clock();
        weight_candidate_swarm_assignments(swarm,
                                           num_timesteps,
                                           delta_time_secs,
                                           threshold,
                                           candidate_assignments);
        ROS_INFO("Total Evaluation time %f",  (clock()-t)/(double) CLOCKS_PER_SEC);
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