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

        // Copy swarm assignment
        agent::SwarmAssignment swarm_assignment_copy = swarm_assignment;
         // Get main assignment
        agent::AgentAssignment &main_usv_assignment = swarm_assignment_copy.at(sim_id);
        // Initialise with original swarm assignment
        std::vector<agent::WeightedSwarmAssignment> candidates={agent::WeightedSwarmAssignment(swarm_assignment, 0.0)};

        bool has_delay_task; // does the other usv have a delay assignment
        int main_usv_task_idx_tmp;
        int other_usv_task_idx_tmp;

        for (auto &assignment_pair : swarm_assignment_copy){
            if (sim_id==assignment_pair.first){
                // Make sure it doesn't exchange tasks with itself
                continue;
            }
            has_delay_task=false;
            agent::AgentAssignment &other_usv_assignment = assignment_pair.second;
            // Check if other usv has a delay assignment
            for(const auto &other_usv_task : other_usv_assignment){
                if(other_usv_task.task_type==agent::TaskType::Delay){
                    has_delay_task=true;
                }
            }
            for(auto &main_usv_task : main_usv_assignment){
                for(auto &other_usv_task : other_usv_assignment){
                    if(main_usv_task.task_type==other_usv_task.task_type){
                        // Exchange Task
                        main_usv_task_idx_tmp = main_usv_task.task_idx;
                        other_usv_task_idx_tmp = other_usv_task.task_idx;

                        exchange_assignment(&main_usv_task.task_idx, &other_usv_task.task_idx);

                        candidates.emplace_back(swarm_assignment_copy, 0.0);
                        // Reset
                        main_usv_task.task_idx = main_usv_task_idx_tmp;
                        other_usv_task.task_idx = other_usv_task_idx_tmp;
                   }else{
                        // Share Task
                        if(main_usv_task.task_type==agent::TaskType::Delay){
                            if(!has_delay_task){
                                other_usv_assignment.push_back(main_usv_task);
                                candidates.emplace_back(swarm_assignment_copy, 0.0);
                                other_usv_assignment.pop_back(); // Reset
                            }
                        }
                   }
                }
            }
        }
    return candidates;
    }

    void run_batch_simulations(agent::USVSwarm swarm,
                              int num_samples,
                              int num_timesteps,
                              double delta_time_secs,
                              double threshold,
                              double &weight){
        agent::AgentType agent_type;
        agent::AgentCommand command;
        agent::MotionGoal usv_motion_goal;
        agent::MotionGoal intruder_motion_goal;
         std::vector<agent::AgentState> obstacle_states;
        if (swarm.get_usv_ids().empty()) ROS_ERROR("USV MAP EMPTY 1");

        std::map<int, agent::AgentType> agent_sim_id_map;
        swarm.get_agent_sim_id_map(agent_sim_id_map);
        if (agent_sim_id_map.empty()) ROS_ERROR("AGENT MAP EMPTY");
        if (swarm.get_usv_ids().empty()) ROS_ERROR("USV MAP EMPTY 2");

        int timestep=0;
        double min_dist_to_asset=-1;
        double dist_to_asset=-1;
        bool all_terminated=false;

        agent::USVSwarm swarm_samples[num_samples];
        bool sample_terminated[num_samples];
        for (int i=0; i<num_samples; i++){
            sample_terminated[i]=false;
            swarm_samples[i] = swarm;
            swarm_samples[i].sample_intruders();
        }
        agent::USVSwarm swarm_sample;
        while(!all_terminated){

            for (int i=0; i<num_samples; i++){
               swarm_sample =  swarm_samples[i];

            }
        }

    }

    void run_swarm_simulation(agent::USVSwarm swarm,
                              int num_timesteps,
                              double delta_time_secs,
                              double threshold,
                              double &weight) {
        bool use_model=false;
       int sim_id;
       agent::AgentType agent_type;
       agent::AgentCommand command;
       agent::MotionGoal usv_motion_goal;
       agent::MotionGoal intruder_motion_goal;
        std::vector<agent::AgentState> obstacle_states;
       if (swarm.get_usv_ids().empty()) ROS_ERROR("USV MAP EMPTY 1");

       std::map<int, agent::AgentType> agent_sim_id_map;
       swarm.get_agent_sim_id_map(agent_sim_id_map);
       if (agent_sim_id_map.empty()) ROS_ERROR("AGENT MAP EMPTY");
       if (swarm.get_usv_ids().empty()) ROS_ERROR("USV MAP EMPTY 2");

       int timestep=0;
       double min_dist_to_asset=-1;
       double dist_to_asset=-1;
       bool terminated=false;
       time_t sample_t= std::clock();
        std::map<int, agent::AgentCommand> intruder_commands_map;
       if (use_model){
           time_t batch_t= std::clock();
           swarm_control::get_batch_intruder_commands(swarm, intruder_commands_map);
           ROS_INFO("Batch query time %f", (std::clock()-batch_t)/(double) CLOCKS_PER_SEC);
       }
       // ROS_INFO("%s", agent::swarm_assignment_to_string(swarm.get_swarm_assignment()).c_str());
       int count_since_shuffle = 0;
       while (!terminated) {
           // std::cout << "ID "<< std::this_thread::get_id();
           min_dist_to_asset = 1000000;
           time_t update_t= std::clock();
           swarm.get_obstacle_states(obstacle_states);
           if(count_since_shuffle>10) {
               swarm.swap_around_observation_tasks();
               count_since_shuffle = 0;
           }
           for (const auto &sim_id_type_pair : agent_sim_id_map) {
               sim_id = sim_id_type_pair.first;
               agent_type = sim_id_type_pair.second;
               if (agent_type == agent::USV) {
                   time_t usv_t= std::clock();
                   swarm_control::get_next_usv_command_by_id(sim_id,
                                                             swarm,
                                                             usv_motion_goal,
                                                             command);
                   // ROS_INFO("USV %d COMMAND (%f, %f)", sim_id, command.delta_heading, command.delta_speed);
                   swarm.command_usv_forward_by_id(sim_id,
                                                   command,
                                                   delta_time_secs);
//                   ROS_INFO("USV sample time %f", (std::clock()-usv_t)/(double) CLOCKS_PER_SEC);
               } else if (agent_type == agent::Intruder) {
                   time_t intruder_t= std::clock();
                   agent::ObservedIntruderAgent intruder = swarm.get_intruder_estimate_by_id(
                           sim_id_type_pair.first);
                   if(!intruder.is_threat()){
                       command = agent::AgentCommand();

                       collision_avoidance::correct_command(intruder, obstacle_states, command);
                   }
                   else{
                       swarm_control::get_next_intruder_command_by_id(sim_id,
                                                                  swarm,
                                                                  intruder_motion_goal,
                                                                  command);
                   }
//                   ROS_INFO("INTRUDER %d COMMAND (%f, %f)", sim_id, command.delta_heading, command.delta_speed);
                   swarm.command_intruder_forward_by_id(sim_id,
                                                        command,
                                                        delta_time_secs);
                   swarm_tools::Point2D intruder_position = intruder.get_state().get_position();
//                   ROS_INFO("INTRUDER %d Position (%f, %f)", sim_id, intruder_position.x, intruder_position.y);
                   dist_to_asset = swarm_tools::euclidean_distance(intruder_position,
                                                                   swarm.get_asset_estimate().get_position());
                   if (min_dist_to_asset > dist_to_asset) {
                       min_dist_to_asset = dist_to_asset;
                       weight = min_dist_to_asset / intruder.get_speed();
                   }
//                   ROS_INFO("Intruder sample time %f", (std::clock()-intruder_t)/(double) CLOCKS_PER_SEC);

                   if (min_dist_to_asset < threshold) {
                       terminated = true;
                       break;
                   };
               }
           }
           // ROS_INFO("Update time %f", (std::clock()-update_t)/(double) CLOCKS_PER_SEC);
//           ROS_INFO("TIMESTEP %d", timestep);
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
        bool use_threads=false;
        int num_samples=50;
        try {
            ROS_INFO("Evaluating swarm assignment:");
            ROS_INFO("%s", agent::swarm_assignment_to_string(swarm_assignment).c_str());
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
                ROS_INFO("WEIGHT: %f", sam_weight);
                total_weight += sam_weight;
            }
            weight = total_weight/(double) num_samples;
            ROS_INFO("WEIGHT: %f, %f", weight, total_weight);
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
        bool use_threads=false;
        double weights[candidate_assignments.size()];
        for (auto &assignment_pair : candidate_assignments){
            // time_t t= std::clock();
            if(use_threads) {
                ROS_INFO("STARTING THREAD");
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
        agent::USVSwarm swarm_cp=swarm;
        swarm_cp.swap_around_observation_tasks();
        agent::SwarmAssignment assignment;
        swarm_cp.get_swarm_assignment(assignment);
        auto candidate_assignments = generate_assignment_candidates(sim_id, assignment, communication_map);
        ROS_INFO("Generation time %f", (std::clock()-t)/(double) CLOCKS_PER_SEC);
        t= std::clock();
        weight_candidate_swarm_assignments(swarm_cp,
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