#include "usv_swarm.h"
#include "motion_goal_control.h"
#include "collision_avoidance.h"
#include <vector>
#include <assert.h>
namespace agent{

    int USVSwarm::get_num_usvs() const{
        return usv_map.size();
    }
    IntruderAgent USVSwarm::get_intruder_estimate_by_id(int intruder_id) const{
        return intruder_map.at(intruder_id);
    }
    USVAgent USVSwarm::get_usv_estimate_by_id(int usv_id) const{
        return usv_map.at(usv_id);
    }

    AssetAgent USVSwarm::get_asset_estimate() const{
        return asset;
    }
    std::vector<IntruderAgent> USVSwarm::get_intruder_estimates() const{
        std::vector<IntruderAgent>  intruders;
        for (const auto &intruder_pair : intruder_map){
            intruders.push_back(intruder_pair.second);
        }
        return intruders;
    }

    std::vector<USVAgent> USVSwarm::get_usv_estimates() const{
        std::vector<USVAgent>  usvs;
        for (const auto &usv_pair : usv_map){
            usvs.push_back(usv_pair.second);
        }
        return usvs;
    }

    SwarmAssignment USVSwarm::get_swarm_assignment() const{
        SwarmAssignment assignment_map;
        for (auto const &usv_pair: usv_map){
            assignment_map[usv_pair.first] = usv_pair.second.get_current_assignment();
        }
        return assignment_map;
    }

    std::vector<AgentState> USVSwarm::get_obstacle_states() const{
        std::vector<AgentState> obstacle_states;
        for (const auto &usv_pair : usv_map){
            obstacle_states.push_back(usv_pair.second.get_state());
        }

        for (const auto &intruder_pair : intruder_map){
            obstacle_states.push_back(intruder_pair.second.get_state());
        }
        return obstacle_states;       
    }

    std::map<int, AgentType> USVSwarm::get_agent_sim_id_map() const{
        std::map<int, AgentType> sim_id_map;

        for (const auto &usv_pair : usv_map){
            sim_id_map[usv_pair.second.get_sim_id()] = USV;
        }

        for (const auto &intruder_pair : intruder_map){
            sim_id_map[intruder_pair.second.get_sim_id()] = Intruder;
        }
        return sim_id_map;   
    }

    void USVSwarm::command_usv_forward_by_id(int usv_id,
                                             const AgentCommand &command,
                                             double delta_time_secs){
        
        usv_map[usv_id].command_agent_forward(command,
                                              delta_time_secs);
    }
    void USVSwarm::command_intruder_forward_by_id(int intruder_id,
                                                  const AgentCommand &command,
                                                  double delta_time_secs){
        
        intruder_map[intruder_id].command_agent_forward(command,
                                                        delta_time_secs);
    }



    void USVSwarm::update_estimates(const std::map<int,
                                        agent::AgentState> &usv_state_map,
                                    const std::map<int,
                                        agent::AgentState> &intruder_state_map
                                    )
    {
        for (auto &intruder_pair : intruder_state_map){
            auto i = intruder_map.find(intruder_pair.first);
            if (i == intruder_map.end()){
                intruder_map[intruder_pair.first] = IntruderAgent(intruder_pair.second);
            }else{
                update_intruder_state_estimate(intruder_pair.second);
            }
        }

        for (auto &usv_pair : usv_state_map){
            auto i = this->usv_map.find(usv_pair.first);
            if (i == usv_map.end()){
                usv_map[usv_pair.first] = USVAgent(usv_pair.second);
            }else{
                update_usv_state_estimate(usv_pair.second);
            }
        }
    }

    void USVSwarm::update_estimates(const std::map<int,
                                        agent::USVAgent> &new_usv_map,
                                    const std::map<int,
                                        agent::IntruderAgent> &new_intruder_map
                                    )
    {
        for (auto &intruder_pair : new_intruder_map){
            auto i = intruder_map.find(intruder_pair.first);
            if (i == intruder_map.end()){
                intruder_map[intruder_pair.first] = IntruderAgent(intruder_pair.second);
            }else{
                update_intruder_estimate(intruder_pair.second);
            }
        }

        for (auto &usv_pair : new_usv_map){
            auto i = this->usv_map.find(usv_pair.first);
            if (i == usv_map.end()){
                usv_map[usv_pair.first] = USVAgent(usv_pair.second);
            }else{
                update_usv_estimate(usv_pair.second);
            }
        }
    }

    void USVSwarm::update_usv_state_estimate(const AgentState &usv_state){
        usv_map[usv_state.sim_id].set_state(usv_state);
    }

    void USVSwarm::update_usv_estimate(const USVAgent &usv){
        usv_map[usv.get_sim_id()].copy(usv);
    }

    void USVSwarm::update_intruder_state_estimate(const AgentState &intruder_state){
        intruder_map[intruder_state.sim_id].set_state(intruder_state);
    }
    void USVSwarm::update_intruder_estimate(const IntruderAgent &intruder){
        intruder_map[intruder.get_sim_id()].copy(intruder);
    }

    void share_assignment(AgentAssignment &main_usv_assignment,
                          AgentAssignment &other_usv_assignment){
        if (other_usv_assignment.delay_assignment_idx != -1){
            int temp_assignment = main_usv_assignment.delay_assignment_idx;
            main_usv_assignment.delay_assignment_idx = other_usv_assignment.delay_assignment_idx;
            other_usv_assignment.delay_assignment_idx = temp_assignment;
        }
        other_usv_assignment.delay_assignment_idx = main_usv_assignment.delay_assignment_idx;
    }
    void exchange_assignment(AgentAssignment &main_usv_assignment,
                             AgentAssignment &other_usv_assignment){
        
        int temp_assignment = main_usv_assignment.guard_assignment_idx;
        main_usv_assignment.guard_assignment_idx = other_usv_assignment.guard_assignment_idx;
        other_usv_assignment.guard_assignment_idx = temp_assignment;
    }

    std::vector<SwarmAssignment> generate_assignment_candidates(int sim_id,
                                                                const SwarmAssignment &swarm_assignment,
                                                                const std::map<int, bool> &communication_map){

        assert(swarm_assignment.find(sim_id)!=swarm_assignment.end());
        AgentAssignment main_usv_assignment = swarm_assignment.at(sim_id);

        std::vector<SwarmAssignment> candidates;
        for (auto const &assignment_pair : swarm_assignment){
            if (sim_id==assignment_pair.first){
                continue;
            }

            SwarmAssignment swarm_assignment_copy_1 = swarm_assignment;
            AgentAssignment main_usv_assignment_copy_1 = main_usv_assignment;
            AgentAssignment other_usv_assignment_copy_1 = assignment_pair.second;
            share_assignment(main_usv_assignment_copy_1, other_usv_assignment_copy_1);
            swarm_assignment_copy_1[sim_id]=main_usv_assignment_copy_1;
            swarm_assignment_copy_1[assignment_pair.first] = other_usv_assignment_copy_1;
            candidates.push_back(swarm_assignment_copy_1);

            SwarmAssignment swarm_assignment_copy_2 = swarm_assignment;
            AgentAssignment main_usv_assignment_copy_2 = main_usv_assignment_copy_2;
            AgentAssignment other_usv_assignment_copy_2 = assignment_pair.second;
            exchange_assignment(main_usv_assignment_copy_2, other_usv_assignment_copy_2);
            swarm_assignment_copy_2[sim_id]=main_usv_assignment_copy_2;
            swarm_assignment_copy_2[assignment_pair.first] = other_usv_assignment_copy_2;
            candidates.push_back(swarm_assignment_copy_2);
        }
    }

    double evaluate_swarm_assignment(const SwarmAssignment &swarm_assignment,
                                     const USVSwarm &swarm,
                                     int num_timesteps,
                                     double delta_time_secs,
                                     double threshold){

        int sim_id;
        AgentType agent_type;
        USVSwarm swarm_copy = swarm;
        auto agent_sim_id_map = swarm.get_agent_sim_id_map();
        AgentCommand command;
        MotionGoal usv_motion_goal;
        MotionGoal delay_motion_goal;
        MotionGoal guard_motion_goal;
        MotionGoal intruder_motion_goal;

        int timestep=0;

        while(timestep < num_timesteps){
            for (const auto &sim_id_type_pair : agent_sim_id_map){
                sim_id = sim_id_type_pair.first;
                agent_type = sim_id_type_pair.second;
                if (agent_type == USV){ 
                    get_next_usv_command_by_id(sim_id,
                                               swarm,
                                               usv_motion_goal,
                                               guard_motion_goal,
                                               delay_motion_goal,
                                               command);
                    swarm_copy.command_usv_forward_by_id(sim_id,
                                                         command,
                                                         delta_time_secs);
                }
                else if (agent_type == Intruder){
                    swarm.get_next_intruder_command_by_id(sim_id,
                                                          intruder_motion_goal,
                                                          command);
                    swarm_copy.command_intruder_forward_by_id(sim_id,
                                                              command,
                                                              delta_time_secs);
                    IntruderAgent intruder = swarm_copy.get_intruder_estimate_by_id(sim_id_type_pair.first);
                    double dist_to_asset = swarm_tools::euclidean_distance(intruder.get_state().get_position(),
                                                                           swarm.get_asset_estimate().get_position());
                    if(dist_to_asset<threshold) return timestep;
                }
            }
            timestep++;
        }
        return timestep;
    }
    std::vector<std::pair<SwarmAssignment*, double>> evaluate_candidate_swarm_assignments(std::vector<SwarmAssignment> candidate_assignments,
                                                                                          USVSwarm swarm,
                                                                                          int num_timesteps,
                                                                                          double delta_time_secs,
                                                                                          double threshold){
        std::vector<std::pair<SwarmAssignment*, double>> weighted_assignments;
        for (SwarmAssignment &assignment : candidate_assignments){
            double weight = evaluate_swarm_assignment(assignment,
                                                      swarm,
                                                      num_timesteps,
                                                      delta_time_secs,
                                                      threshold);
            weighted_assignments.push_back(std::pair<SwarmAssignment*, double>(&assignment, weight));
        }
        return weighted_assignments;
    }

    std::vector<std::pair<SwarmAssignment*, double>> generate_weighted_candidate_assignments(int sim_id,
                                                                                             USVSwarm &swarm,
                                                                                             std::map<int, bool> &communication_map,
                                                                                             int num_timesteps,
                                                                                             double delta_time_secs,
                                                                                             double threshold){
        auto candidate_assignments = generate_assignment_candidates(sim_id, swarm.get_swarm_assignment(), communication_map);
        return evaluate_candidate_swarm_assignments(candidate_assignments,
                                                    swarm,
                                                    num_timesteps,
                                                    delta_time_secs,
                                                    threshold);
    }

    int USVSwarm::get_next_usv_command_by_id(int usv_id,
                                             MotionGoal &motion_goal,
                                             MotionGoal &guard_motion_goal,
                                             MotionGoal &delay_motion_goal,
                                             AgentCommand &command) const{

        int num_usvs = get_num_usvs();
        USVAgent usv = get_usv_estimate_by_id(usv_id);
        AssetAgent asset = get_asset_estimate();

        swarm_control::get_motion_goals_from_assignment(usv_id,
                                                        *this,
                                                        asset,
                                                        delay_motion_goal,
                                                        guard_motion_goal,
                                                        motion_goal);

        swarm_control::get_command_from_motion_goal(usv.get_state(),
                                                    usv.get_constraints(),
                                                    motion_goal,
                                                    command);

        collision_avoidance::correct_command(usv,
                                             get_obstacle_states(),
                                             command);
        return 0;
    }

    int USVSwarm::get_next_intruder_command_by_id(int intruder_id,
                                                  MotionGoal &intruder_motion_goal,
                                                  AgentCommand &command) const{
    
        IntruderAgent intruder = get_intruder_estimate_by_id(intruder_id);
        swarm_control::intruder_motion_goal(
            get_asset_estimate().get_state(),
            intruder_motion_goal);

        swarm_control::get_command_from_motion_goal(intruder.get_state(),
                                                    intruder.get_constraints(),
                                                    intruder_motion_goal,
                                                    command);

        collision_avoidance::correct_command(intruder,
                                             get_obstacle_states(),
                                             command);
    }
}