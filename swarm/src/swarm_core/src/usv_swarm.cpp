#include "usv_swarm.h"
#include <assert.h>
namespace agent{

    int USVSwarm::get_num_usvs() const{
        return usv_map.size();
    }
    IntruderAgent USVSwarm::get_intruder_estimate_by_id(int intruder_id) const{
        assert(intruder_map.find(intruder_id)!=intruder_map.end());
        return intruder_map.at(intruder_id);
    }
    USVAgent USVSwarm::get_usv_estimate_by_id(int usv_id) const{
        assert(usv_map.find(usv_id)!=usv_map.end());
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
        for (const std::pair<int, USVAgent> &usv_pair : usv_map){
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

    void USVSwarm::update_swarm_assignment(const SwarmAssignment &swarm_assignment){
        for(const auto &assignment_pair : swarm_assignment){
            update_agent_assignment_by_id(assignment_pair.first, assignment_pair.second);
        }
    }

    void USVSwarm::update_agent_assignment_by_id(int sim_id,
                                                 const AgentAssignment &assignment){
        usv_map[sim_id].set_current_assignment(assignment);
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

}