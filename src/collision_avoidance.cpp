#include "swarm_tools.h"
#include "collision_avoidance.h"
#include <vector>
#include <algorithm>
#include <cmath>

using namespace collision_avoidance;


int correct_command(
    const AgentState &agent_state,
    AgentCommand &command,
    const std::vector<swarm_tools::PointInterval> &edge_points,
    const AgentConstraints &constraints,
    const double &max_distance,
    const double &max_angle_rad,
    const double &aggression
){
    std::vector<swarm_tools::AngleInterval> safe_intervals;
    bool flag = get_safe_intervals(agent_state,
                                   edge_points,
                                   max_distance,
                                   max_angle_rad,
                                   safe_intervals);

    if (!flag){
        // Fan is totally blocked
        if (command.delta_heading<0){
            command.delta_heading = -max_angle_rad;
        }else{
            command.delta_heading = max_angle_rad;
        }
        return -1; // Indicates the fan was completely blocked 
    }

    // restrict command using differential constraints
    double delta_heading = swarm_tools::clip(command.delta_heading,
                                             -max_angle_rad,
                                             max_angle_rad);
 
    if (is_command_safe(command.delta_heading, safe_intervals)){
        return 0; // Indicates no change was made.
    }

    // Get largest interval
    int largest_interval_index = largest_safe_interval(safe_intervals);
    auto largest_interval = safe_intervals[largest_interval_index];

    auto l_theta_rad = largest_interval.l_theta_rad;
    auto r_theta_rad = largest_interval.r_theta_rad;

    // Get the weighted mid point of the interval.
    if (std::abs(l_theta_rad-delta_heading) < std::abs(r_theta_rad-delta_heading)){
        command.delta_heading = aggression*l_theta_rad + (1-aggression)*r_theta_rad;
    }else{
        command.delta_heading = aggression*r_theta_rad + (1-aggression)*l_theta_rad;
    }

    // restrict command using differential constraint
    command.delta_heading = swarm_tools::clip(command.delta_heading, -max_angle_rad, max_angle_rad);
    return 1;
}

bool is_command_safe(
    const double &delta_heading,
    const std::vector<swarm_tools::AngleInterval> safe_intervals
){
    for (const swarm_tools::AngleInterval interval : safe_intervals){
        if (interval.contains(delta_heading)){
            return true;
        }
    }
    return false;
}

int largest_safe_interval(
    const std::vector<swarm_tools::AngleInterval> safe_intervals
){
    int max_index = -1;
    double max_diff = -1;
    for (auto ip=safe_intervals.begin();ip != safe_intervals.end(); ip++){
        double diff = std::abs(ip->l_theta_rad-ip->r_theta_rad);
        if (diff>=max_diff){
            max_index = (safe_intervals.begin()-ip);
            max_diff = diff;
        }
    }
    return max_index;
}

bool collision_avoidance::collision_check(

    const AgentState &agent_state,
    const swarm_tools::Point2D &left,
    const swarm_tools::Point2D &right,
    const double &max_distance,
    const double &max_angle_rad,
    double &l_theta_rad,
    double &r_theta_rad
){

    swarm_tools::Point2D agent_position = agent_state.position();
    swarm_tools::Point2D mid_point = swarm_tools::mid_point(left, right);
    
    // Get distance to mid point of object
    double distance = swarm_tools::euclidean_distance(agent_position, mid_point);

    // Get the relative angle to the edge points of object
    l_theta_rad = swarm_tools::relative_angle_between_points(agent_position, left, agent_state.heading);
    r_theta_rad = swarm_tools::relative_angle_between_points(agent_position, right, agent_state.heading);

    bool is_in_fan = in_fan(distance, l_theta_rad, r_theta_rad, max_distance, max_angle_rad);

    if (is_in_fan){
        l_theta_rad = std::max(l_theta_rad, max_angle_rad);
    }

    return is_in_fan;
}//collision_check

bool collision_avoidance::in_fan(
    const double &distance,
    const double &l_theta_rad,
    const double &r_theta_rad,
    const double &max_distance,
    const double &max_angle_rad
){
    // Check if angles are in range
    bool left_in_range = swarm_tools::PI/2 <= l_theta_rad && l_theta_rad <= tools::PI/2;
    bool right_in_range = swarm_tools::PI/2 <= r_theta_rad && l_theta_rad <= tools::PI/2;

    if (!(left_in_range && right_in_range)){
        return false; // Angles are out of range
    }
 
    bool flag;
    if (l_theta_rad >= 0 and r_theta_rad<=0){
        flag = true;
    }
    else{
        flag = l_theta_rad >= r_theta_rad;
    }
}//in_fan


int collision_avoidance::get_safe_intervals(
    const AgentState agent_state,
    const std::vector<swarm_tools::PointInterval> edge_intervals,
    const double max_distance,
    const double max_angle_rad,
    std::vector<swarm_tools::AngleInterval> safe_intervals
){
    std::vector<swarm_tools::AngleInterval> occupied_intervals;
    
    for (auto point_interval : edge_intervals){
        const auto left_point = point_interval.left_point;
        const auto right_point = point_interval.right_point;

        double l_theta;
        double r_theta;

        bool flag = collision_check(agent_state,
                                   left_point,
                                   right_point,
                                   max_distance,
                                   max_angle_rad,
                                   l_theta,
                                   r_theta);
        if (flag){
            continue;
        }else{
            const swarm_tools::AngleInterval interval = {l_theta, r_theta};
            occupied_intervals.push_back(interval);
        }
    
    std::sort(occupied_intervals); //TODO
    swarm_tools::AngleInterval safe_interval;
    double right_theta_rad=max_angle_rad;
    for ( auto ip = occupied_intervals.begin(); ip == occupied_intervals.end(); ip++){
        if (ip==occupied_intervals.end()){
            if(right_theta_rad > -max_angle_rad){
                safe_interval = {right_theta_rad, -max_angle_rad};
                safe_intervals.push_back(safe_interval);
            }
        }else{
            if(right_theta_rad < ip->l_theta_rad){
                right_theta_rad = ip->l_theta_rad;
            }else{
                safe_interval = {right_theta_rad, ip->l_theta_rad};
                safe_intervals.push_back(safe_interval);
                right_theta_rad = ip->r_theta_rad;
            }
        }
    }
        
    }
}
