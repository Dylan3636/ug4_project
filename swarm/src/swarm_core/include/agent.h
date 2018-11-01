#include "swarm_tools.h"

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
};

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