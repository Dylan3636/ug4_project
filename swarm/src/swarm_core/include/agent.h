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
    double sim_id;

    swarm_tools::Point2D position() const;
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

int get_left_and_right_points();
}
#endif