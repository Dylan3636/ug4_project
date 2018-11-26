#include "swarm_tools.h"
#include <vector>
#include <map>

#ifndef AGENT_H
#define AGENT_H

namespace agent{
enum AgentType{
    USV,
    Intruder,
    Static,
    Asset
};

struct AgentState
{
    double x;
    double y;
    double speed;
    double heading;
    double radius;
    int sim_id;

    swarm_tools::Point2D position() const{
        return swarm_tools::Point2D{this->x, this->y};
    }
};

struct AgentCommand
{
    double delta_speed;
    double delta_heading;
};

struct AgentConstraints
{
    double max_speed;
    double max_delta_speed;
    double max_delta_heading;
    double max_radar_radius;
    double max_radar_width_rad;
    double aggression; 
};

struct AgentAssignment
{
    int* delay_assignment_idx_ptr;
    int* guard_assignment_idx_ptr;
};

class BaseAgent
{
    public:
        AgentState state;
        AgentConstraints constraints;
        AgentType type;

        int get_sim_id(){
            return this->state.sim_id;
        }
        swarm_tools::Point2D position() const{
            return state.position();
        }
};

class IntruderAgent : public BaseAgent
{   
};

struct AssetAgent : public BaseAgent
{
};


class USVAgent : public BaseAgent
{

    void update_intruder_state_estimate(const AgentState &intruder_state);

    public:
        std::map<int, IntruderAgent> intruder_map;
        AgentAssignment current_assignment;
        IntruderAgent get_intruder_estimate_by_id(int intruder_id);

        void update_estimates(const std::map<int, AgentState> &usv_state_map,
                              const std::map<int, AgentState> &intruder_state_map);
        std::vector<IntruderAgent> get_intruder_estimates();

};


USVAgent default_usv_agent(const AgentState &state);

IntruderAgent default_intruder_agent(const AgentState &state);

AssetAgent default_asset_agent(const AgentState &state);
struct MotionGoal{
    double x;   
    double y;
    double speed;
    double heading_rad;
    swarm_tools::Point2D position() const{
        return swarm_tools::Point2D{this->x, this->y};
    }
};

int get_left_and_right_points();
}
#endif