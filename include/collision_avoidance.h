#include "agent.h"
#include "swarm_tools.h"
#include <vector>

namespace collision_avoidance{

bool collision_check(
    const AgentState &agent_state,
    const swarm_tools::Point2D &left,
    const swarm_tools::Point2D &right,
    const double &max_distance,
    const double &max_angle,
    double &l_theta,
    double &r_theta
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


int get_safe_intervals(
    const AgentState agent_state,
    const std::vector<swarm_tools::PointInterval>,
    const double max_angle_rad,
    const double max_distance,
    std::vector<swarm_tools::AngleInterval> occupied_intervals
);
}