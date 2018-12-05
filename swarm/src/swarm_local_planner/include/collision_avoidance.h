#include "agent.h"
#include "swarm_tools.h"
#include <vector>


#ifndef CA_H
#define CA_H
namespace collision_avoidance{

template<typename T>
int correct_command(
    const T &agent,
    const std::vector<agent::AgentState> &obstacle_states,
    agent::AgentCommand &command
);

int correct_command(
    const agent::AgentState &agent_state,
    agent::AgentCommand &command,
    const std::vector<swarm_tools::PointInterval> &edge_points,
    const agent::AgentConstraints &constraints,
    const double &max_distance,
    const double &max_angle_rad,
    const double &aggression);

bool collision_check(
    const agent::AgentState &agent_state,
    const swarm_tools::Point2D &left,
    const swarm_tools::Point2D &right,
    const double &max_distance,
    const double &max_angle_rad,
    double &l_theta_rad,
    double &r_theta_rad
);

bool in_fan(
    const double &distance,
    const double &l_theta_rad,
    const double &r_theta_rad,
    const double &max_distance,
    const double &max_angle
);

bool check_angle(
    const double &l_theta_rad,
    const double &r_theta_rad
);

bool is_command_safe(
    const double& delta_heading,
    const std::vector<swarm_tools::AngleInterval>& safe_intervals
);

int largest_safe_interval(
    const std::vector<swarm_tools::AngleInterval>& safe_intervals);

int nearest_safe_interval(
    const std::vector<swarm_tools::AngleInterval>& safe_intervals,
    const double &delta_heading);

int get_safe_intervals(
    const agent::AgentState& agent_state,
    const std::vector<swarm_tools::PointInterval>& edges,
    const double& max_angle_rad,
    const double& max_distance,
    std::vector<swarm_tools::AngleInterval>& safe_intervals
);
}
#endif 