#include "agent.h"
#include "swarm_tools.h"
#include <vector>


#ifndef CA_H
#define CA_H
namespace collision_avoidance{

template<typename T> int correct_command(
    const T &agent,
    const std::vector<agent::AgentState> &obstacle_states,
    agent::AgentCommand &command
);

template<typename T> int correct_command(
        const T &agent,
    agent::AgentCommand &command,
    const std::vector<swarm_tools::PointInterval> &edge_points
    );

bool collision_check(
    const agent::AgentState &agent_state,
    const swarm_tools::Point2D &left,
    const swarm_tools::Point2D &right,
    double max_distance,
    double max_angle_rad,
    double &l_theta_rad,
    double &r_theta_rad
);

bool in_fan(
    double distance,
    double l_theta_rad,
    double r_theta_rad,
    double max_distance,
    double max_angle
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

long nearest_safe_interval(
    const std::vector<swarm_tools::AngleInterval>& safe_intervals,
    const double &delta_heading);

int get_safe_intervals(
    const agent::AgentState& agent_state,
    const std::vector<swarm_tools::PointInterval>& edges,
    double max_distance,
    double max_angle_rad,
    std::vector<swarm_tools::AngleInterval>& safe_intervals
);
}
#endif 