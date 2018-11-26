#include <map>
#include <vector>
#include "agent.h"
#include "swarm_tools.h"

namespace agent{

    IntruderAgent USVAgent::get_intruder_estimate_by_id(int intruder_id){
        return this->intruder_map[intruder_id];
    }

    void USVAgent::update_estimates(const std::map<int, agent::AgentState> &usv_state_map,
                                    const std::map<int, agent::AgentState> &intruder_state_map
                                    )
{
    for (auto &intruder_pair : intruder_state_map){
        auto i = this->intruder_map.find(intruder_pair.first);
        if (i == intruder_map.end()){
            intruder_map[intruder_pair.first] = default_intruder_agent(intruder_pair.second);
        }else{
            this->update_intruder_state_estimate(intruder_pair.second);
        }
    }

}
    void USVAgent::update_intruder_state_estimate(const AgentState &intruder_state){
        this->intruder_map[intruder_state.sim_id].state = intruder_state;
    } // update_intruder_state_estimate

    std::vector<IntruderAgent> USVAgent::get_intruder_estimates(){
        std::vector<IntruderAgent>  intruders;
        for (const auto &intruder_pair : intruder_map){
            intruders.push_back(intruder_pair.second);
        }
        return intruders;
    } // get_intruder_estimates

    IntruderAgent default_intruder_agent(const AgentState &state){
        IntruderAgent intruder;
        intruder.state = state;
        intruder.constraints = {30, 5, swarm_tools::PI/2, 100, swarm_tools::PI, 0.5};
        return intruder;
    } // default_intruder_agent

    USVAgent default_usv_agent(const AgentState &state){
        USVAgent usv;
        usv.state=state;
        usv.constraints = {35, 5, swarm_tools::PI/2, 100, swarm_tools::PI, 0.9};
        return usv;
    } // default_usv_agent

    AssetAgent default_asset_agent(const AgentState &state){
        AssetAgent asset;
        asset.state = state;
        return asset;
    }

}