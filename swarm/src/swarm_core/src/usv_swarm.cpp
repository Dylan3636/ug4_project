#include "usv_swarm.h"
#include "boost/format.hpp"
#include <assert.h>
#include "ros_swarm_tools.h"
#include "swarm_threat_detection/ThreatDetection.h"
#include "swarm_threat_detection/batchIntruderCommands.h"
namespace agent{

    void USVSwarm::reset(){
        usv_map.clear();
        intruder_map.clear();
    }
    int USVSwarm::get_num_usvs() const{
        return usv_map.size();
    }
    ObservedIntruderAgent USVSwarm::get_intruder_estimate_by_id(int intruder_id) const{
        // assert(intruder_map.find(intruder_id)!=intruder_map.end());
        if(usv_map.empty()){
            ROS_ERROR("Intruder Map is empty!");
            throw std::exception();
        }
        try{
            return intruder_map.at(intruder_id);
        }
        catch (const std::out_of_range &error){
            auto str = boost::format("INTRUDER ID : %d NOT FOUND!") % intruder_id;
            std::cout << str.str() << std::endl;
            throw str.str();
        }
    }
    USVAgent USVSwarm::get_usv_estimate_by_id(int usv_id) const{
        if(usv_map.empty()){
            ROS_ERROR("USV Map is empty!");
            throw std::exception();
        }
        // assert(usv_map.find(usv_id)!=usv_map.end());
        try{
            return usv_map.at(usv_id);
        }
        catch (const std::out_of_range &error){
            ROS_ERROR("USV ID : %d NOT FOUND!\n %s", usv_id, error.what());
            ROS_ERROR("USV Map contains");
            for (const auto pair : usv_map){
                ROS_ERROR("USV ID %d", pair.first);
            }
            throw std::exception();
        }
    }

    AssetAgent USVSwarm::get_asset_estimate() const{
        return asset;
    }
    std::vector<ObservedIntruderAgent> USVSwarm::get_intruder_estimates() const{
        std::vector<ObservedIntruderAgent>  intruders;
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
    AgentAssignment USVSwarm::get_assignment_by_id(int usv_id) const{
        auto str = boost::format("USV ID : %d NOT FOUND!") % usv_id;
        assert(usv_map.find(usv_id)!=usv_map.end() && str.str().c_str());
        return usv_map.at(usv_id).get_current_assignment();
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
    std::vector<int> USVSwarm::get_usv_ids() const{
        std::vector<int> usv_ids;
        for (const std::pair<int, USVAgent> &usv_pair : usv_map){
            usv_ids.push_back(usv_pair.first);
        }
        return usv_ids;
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
        assert(usv_map.find(sim_id)!=usv_map.end());
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
        
        // ROS_INFO("DELTA TIME SEC %f", delta_time_secs);
        // ROS_INFO("(%f, %f)", command.delta_speed, command.delta_heading);
        auto &intruder = intruder_map[intruder_id];
        // ROS_INFO("BEFORE: (x, y): (%f, %f)",intruder.get_x(), intruder.get_y());
        intruder.command_agent_forward(command,
                                       delta_time_secs);
        // ROS_INFO("AFTER: (x, y): (%f, %f)",intruder.get_x(), intruder.get_y());
    }

    bool USVSwarm::switch_observe_to_delay_task(int intruder_id){
       auto sorted_usvs = sort_usvs_by_weighted_distance_to_point(get_intruder_estimate_by_id(intruder_id).get_position());
       bool switched=false;
       for(int usv_id : sorted_usvs){
           usv_map[usv_id].remove_observe_assignment(intruder_id);
           if(!switched){
               switched = usv_map[usv_id].switch_observe_to_delay_assignment(intruder_id);
               ROS_ERROR("I SWITCHED %d", switched);
           }
       }
       return switched;
    }


    void USVSwarm::update_intruder_estimates(const std::map<int,
                                             agent::AgentState> &intruder_state_map)
    {
        for (auto &intruder_pair : intruder_state_map){
            auto i = intruder_map.find(intruder_pair.first);
            if (i == intruder_map.end()){
                intruder_map[intruder_pair.first] = ObservedIntruderAgent(intruder_pair.second);
                assign_intruder_to_usv(intruder_map[intruder_pair.first]);
            }else{
                update_intruder_state_estimate(intruder_pair.second);
               
            }
        }
    }

    bool USVSwarm::update_intruder_threat_estimate(const AgentState &intruder_state){
        swarm_threat_detection::ThreatDetection srv;
        double dist_to_intruder = swarm_tools::euclidean_distance(
            get_usv_estimate_by_id(get_main_usv_id()).get_position(),
            intruder_state.get_position());
        srv.request.dist_to_intruder = dist_to_intruder;
        srv.request.intruder_id = intruder_state.sim_id;

        if(threat_detection_client.call(srv)){
            ROS_INFO("Intruder Threat Detection Service Successful!");
            ROS_INFO("For Intruder %d : (%d, %f)", intruder_state.sim_id, srv.response.threat_alert, srv.response.threat_probability);
            bool new_threat = (!get_intruder_estimate_by_id(intruder_state.sim_id).is_threat() && srv.response.threat_alert);
            ROS_INFO("New threat_classification %d", new_threat);
            intruder_map[intruder_state.sim_id].set_threat_estimate(srv.response.threat_alert, srv.response.threat_probability);
            return new_threat;
        }else{
            ROS_INFO("Intruder Threat Detection Failed!");
            ROS_ERROR("Intruder Threat Detection Failed!");
            return false;
        }
        
    }

    void USVSwarm::update_usv_estimates(const std::map<int,
                                        agent::AgentState> &usv_state_map)
    {
        for (auto &usv_pair : usv_state_map){
            auto i = this->usv_map.find(usv_pair.first);
            if (i == usv_map.end()){
                usv_map[usv_pair.first] = USVAgent(usv_pair.second);
                assign_guard_task_to_usv(usv_pair.first);
            }else{
                update_usv_state_estimate(usv_pair.second);
            }
        }
    }

    void USVSwarm::update_estimates(const std::map<int,
                                        agent::AgentState> &usv_state_map,
                                    const std::map<int,
                                        agent::AgentState> &intruder_state_map
                                    )
        {
        update_usv_estimates(usv_state_map);
        update_intruder_estimates(intruder_state_map);
    }

    std::vector<int> USVSwarm::sort_usvs_by_distance_to_point(const swarm_tools::Point2D &point) const{
        return sort_usvs_by_distance_to_point(get_usv_estimates(), point);
    }

    std::vector<int> USVSwarm::sort_usvs_by_distance_to_point(
            const std::vector<USVAgent> &usvs,
            const swarm_tools::Point2D &point) const{

        std::vector<std::pair<int, double>> usv_distance_id_pairs;
        for (const auto &usv : usvs){
            double distance = swarm_tools::euclidean_distance(usv.get_position(), point);
            usv_distance_id_pairs.emplace_back(usv.get_sim_id(), distance);
        }
        std::sort(usv_distance_id_pairs.begin(),
                  usv_distance_id_pairs.end(),
                  [](const std::pair<int, double> &usv_pair_1,
                     const std::pair<int, double> &usv_pair_2){
                        return usv_pair_1.second<usv_pair_2.second;
                     } );
        std::vector<int> sorted_usv_ids;
        for (const auto &distance_pair : usv_distance_id_pairs){
            sorted_usv_ids.push_back(distance_pair.first);
        }
        return sorted_usv_ids;
    }

    std::vector<int> USVSwarm::sort_usvs_by_weighted_distance_to_point(const swarm_tools::Point2D &point) const{
        return sort_usvs_by_weighted_distance_to_point(get_usv_estimates(), point);
    }

    std::vector<int> USVSwarm::sort_usvs_by_weighted_distance_to_point(
            const std::vector<USVAgent> &usvs,
            const swarm_tools::Point2D &point) const{

        double occupied_param = 100;
        double distance;
        double occupation_bonus;
        std::vector<std::pair<int, double>> usv_distance_id_pairs;
        for (const auto &usv : usvs){
            distance = swarm_tools::euclidean_distance(usv.get_position(), point);
            occupation_bonus = std::max(0.0,  + occupied_param*(usv.get_current_assignment().size()-1));
            distance += occupation_bonus;
            usv_distance_id_pairs.emplace_back(usv.get_sim_id(), distance);
        }
        std::sort(usv_distance_id_pairs.begin(),
                  usv_distance_id_pairs.end(),
                  [](const std::pair<int, double> &usv_pair_1,
                     const std::pair<int, double> &usv_pair_2){
                        return usv_pair_1.second<usv_pair_2.second;
                     } );
        std::vector<int> sorted_usv_ids;
        for (const auto &distance_pair : usv_distance_id_pairs){
            sorted_usv_ids.push_back(distance_pair.first);
        }
        return sorted_usv_ids;
    }

    int USVSwarm::get_delay_position(int usv_id, int intruder_id) const{
        std::vector<USVAgent> usvs_w_delay_task;
        for(const auto &usv_pair: usv_map){
            if(usv_pair.second.has_delay_task(intruder_id)){
                usvs_w_delay_task.push_back(usv_pair.second);
            }
        }
        assert(!usvs_w_delay_task.empty() && (boost::format("USV %d IS NOT DELAYING INTRUDER %d") % usv_id, intruder_id));
        if(usvs_w_delay_task.size()==1) return 0;

        auto usv_ids = sort_usvs_by_distance_to_point(usvs_w_delay_task, get_intruder_estimate_by_id(intruder_id).get_position());
        uint8_t count = 0;
        for(int id : usv_ids){
            if(usv_id==id) return count;
            count++;
        }
    }

    void USVSwarm::update_estimates(const std::map<int,
                                        agent::USVAgent> &new_usv_map,
                                    const std::map<int,
                                        agent::ObservedIntruderAgent> &new_intruder_map
                                    )
    {
        for (auto &intruder_pair : new_intruder_map){
            auto i = intruder_map.find(intruder_pair.first);
            if (i == intruder_map.end()){
                intruder_map[intruder_pair.first] = ObservedIntruderAgent(intruder_pair.second);
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

        clock_t t = clock();
        bool new_threat_alert = update_intruder_threat_estimate(intruder_state);
        ROS_DEBUG("Update intruder threat estimate time %f", (clock()-t)/(double) CLOCKS_PER_SEC);
        if (new_threat_alert){
            ROS_INFO("Switching Observe Task to Delay Task for Intruder %d", intruder_state.sim_id);
            t = clock();
            bool successful = switch_observe_to_delay_task(intruder_state.sim_id);
            ROS_ERROR("Delay switch time %f", (clock()-t)/(double) CLOCKS_PER_SEC);
            if(successful){
                ROS_INFO("Switch successful!");
                intruder_map[intruder_state.sim_id].set_threat_classification(true);
                block_next_task_allocation=true;
            }else{
                ROS_ERROR("Switch failed!");
            }

        }

    }
    void USVSwarm::update_intruder_estimate(const ObservedIntruderAgent &intruder){
        intruder_map[intruder.get_sim_id()].copy(intruder);
    }
    bool USVSwarm::contains_usv(int usv_id){
        return usv_map.find(usv_id) != usv_map.end();
    }
    bool USVSwarm::contains_intruder(int intruder_id){
        return intruder_map.find(intruder_id) != intruder_map.end();
    }
    void USVSwarm::add_usv(const USVAgent &usv){
        usv_map[usv.get_sim_id()] = USVAgent();
        usv_map[usv.get_sim_id()].copy(usv);
        assign_guard_task_to_usv(usv.get_sim_id());
    }
    void USVSwarm::add_intruder(ObservedIntruderAgent intruder){
        intruder_map[intruder.get_sim_id()] = intruder;
        assign_intruder_to_usv(intruder);
    }
    void USVSwarm::assign_intruder_to_usv(const ObservedIntruderAgent &intruder){
        std::vector<int> sorted_usv_ids = sort_usvs_by_weighted_distance_to_point(intruder.get_position());
        for(const auto usv_id : sorted_usv_ids){
            if(usv_map[usv_id].has_delay_task()){
                continue;
            }
            else{
                usv_map[usv_id].add_observe_task(intruder.get_sim_id());
                return;
            }
        }
    }
    void USVSwarm::assign_guard_task_to_usv(int usv_id){
        usv_map[usv_id].set_guard_assignment(get_num_usvs()-1);
    }
    void USVSwarm::sample_intruders(){
        std::default_random_engine generator;
        for(auto &intruder_pair : intruder_map){
            intruder_pair.second.sample(generator);
        }
    }
//    bool USVSwarm::get_batch_intruder_commands_from_model(std::vector<agent::AgentCommand> &commands){
//        return swarm_control::get_batch_intruder_commands_from_model(get_intruder_estimates(),
//                commands,
//                intruder_model_client);
//
//    }
}