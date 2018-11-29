#include <assert.h>
#include "agent.h"

namespace agent{

    void USVAgent::copy(const USVAgent &usv){
        BaseAgent::copy(usv);
        set_current_assignment(usv.get_current_assignment());
    }
    IntruderAgent USVAgent::get_intruder_estimate_by_id(int intruder_id) const{
        return intruder_map.at(intruder_id);
        
    }
    AgentAssignment USVAgent::get_current_assignment() const{
        return this->current_assignment;
    }
    void USVAgent::set_current_assignment(AgentAssignment assignment){
        this->current_assignment=assignment;
    }

    void USVAgent::update_estimates(const std::map<int,
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
    void USVAgent::update_estimates(const std::map<int,
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
    void USVAgent::update_usv_state_estimate(const AgentState &usv_state){
        if (usv_state.sim_id == get_sim_id()){
            set_state(usv_state);
        }else{
            usv_map[usv_state.sim_id].set_state(usv_state);
        }
    }// update_usv_state_estimate

    void USVAgent::update_usv_estimate(const USVAgent &usv){
        if (usv.get_sim_id() == get_sim_id()){
            copy(usv);
        }
        usv_map[usv.get_sim_id()].copy(usv);
    }// update_usv_state_estimate

    void USVAgent::update_intruder_state_estimate(const AgentState &intruder_state){
        intruder_map[intruder_state.sim_id].set_state(intruder_state);
    } // update_intruder_state_estimate
    void USVAgent::update_intruder_estimate(const IntruderAgent &intruder){
        intruder_map[intruder.get_sim_id()].copy(intruder);
    } // update_intruder_state_estimate

    std::vector<USVAgent> USVAgent::get_usv_estimates() const{
        std::vector<USVAgent>  usvs;
        for (const auto &usv_pair : usv_map){
            if (usv_pair.second.get_sim_id()==get_sim_id()){
                continue;
            }
            usvs.push_back(usv_pair.second);
        }
        return usvs;
    } // get_usv_estimates

    SwarmAssignment USVAgent::get_swarm_assignment() const{
        SwarmAssignment assignment_map;
        for (auto const &usv_pair: usv_map){
            assignment_map[usv_pair.first] = usv_pair.second.get_current_assignment();
        }
        return assignment_map;
    }

    std::vector<AgentState> USVAgent::get_obstacle_states() const{
        std::vector<AgentState> obstacle_states;
        for (const auto &usv_pair : usv_map){
            if (usv_pair.second.get_sim_id()==get_sim_id()){
                continue;
            }
            obstacle_states.push_back(usv_pair.second.get_state());
        }

        for (const auto &intruder_pair : intruder_map){
            obstacle_states.push_back(intruder_pair.second.get_state());
        }
        return obstacle_states;       
    }

    std::vector<IntruderAgent> USVAgent::get_intruder_estimates() const{
        std::vector<IntruderAgent>  intruders;
        for (const auto &intruder_pair : intruder_map){
            intruders.push_back(intruder_pair.second);
        }
        return intruders;
    } // get_intruder_estimates
    
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
    std::vector<SwarmAssignment> generate_assignment_candidates(                                                                        
        int sim_id,
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
    std::vector<double> USVAgent::evaluate_swarm_assignment(const SwarmAssignment &swarm_assignment,
                                                            int timesteps,
                                                            double delta_time_secs){
        std::map<int, USVAgent> usv_map_copy = usv_map;
        std::map<int, IntruderAgent> intruder_map_copy = intruder_map;
        AssetAgent asset_copy = asset;
        for (const auto &usv_assignment_pair: swarm_assignment){
            usv_map_copy[usv_assignment_pair.first].set_current_assignment(usv_assignment_pair.second);
        }

        for (auto &usv_pair : usv_map_copy){
            //usv
        }
        for (auto &intruder_pair : intruder_map_copy){

        }
    }
    //get_next_command()
//    AgentState USVAgent::project_state_forward(int timesteps,
//                                               double delta_time_secs
//                                               ){
//        for (int t=0; t<timesteps; t++){

//        }
        
//    } 

}