#include <iostream>
#include "collision_avoidance.h"
#include "swarm_tools.h"
#include <vector>
#include <algorithm>
#include <cmath>

template<typename T> int collision_avoidance::correct_command(
    const T &agent,
    const std::vector<agent::AgentState> &obstacle_states,
    agent::AgentCommand &command
){
    std::vector<swarm_tools::PointInterval> edge_points;
    for (auto &obstacle_state : obstacle_states){
        if(agent.get_sim_id()==obstacle_state.sim_id) continue;
        swarm_tools::Point2D left_edge;
        swarm_tools::Point2D right_edge;
        swarm_tools::edge_points_of_circle(agent.get_position(),
                                           obstacle_state.get_position(),
                                           obstacle_state.radius,
                                           agent.get_heading(),
                                           left_edge,
                                           right_edge
                                           );
        swarm_tools::PointInterval pi = {left_edge, right_edge};
        edge_points.push_back(pi);
    }
    return collision_avoidance::correct_command(agent,
                                                command,
                                                edge_points
                                                );
}
template int collision_avoidance::correct_command<agent::USVAgent>(const agent::USVAgent &agent,
                                                                   const std::vector<agent::AgentState> &obstacle_states,
                                                                   agent::AgentCommand &command);
template int collision_avoidance::correct_command<agent::ObservedIntruderAgent>(const agent::ObservedIntruderAgent &agent,
                                                                        const std::vector<agent::AgentState> &obstacle_states,
                                                                        agent::AgentCommand &command);
template int collision_avoidance::correct_command<agent::IntruderAgent>(const agent::IntruderAgent &agent,
                                                                        const std::vector<agent::AgentState> &obstacle_states,
                                                                        agent::AgentCommand &command);

template<typename T> int collision_avoidance::correct_command(
    const T &agent,
    agent::AgentCommand &command,
    const std::vector<swarm_tools::PointInterval> &edge_points
){
    agent::AgentState agent_state = agent.get_state();
    agent::AgentConstraints constraints = agent.get_constraints();
    double max_distance = agent.get_max_radar_distance();
    double max_angle_rad = agent.get_max_radar_angle_rad();
    double aggression = agent.get_aggression();
    std::vector<swarm_tools::AngleInterval> safe_intervals;
    int flag = get_safe_intervals(agent_state,
                                  edge_points,
                                  max_distance,
                                  max_angle_rad,
                                  safe_intervals);
    command.delta_heading = swarm_tools::clip(command.delta_heading, -max_angle_rad, max_angle_rad);

    if (safe_intervals.empty()){
        // Fan is totally blocked
        if (command.delta_heading<0){
            command.delta_heading = -max_angle_rad;
        }else{
            command.delta_heading = max_angle_rad;
        }
        command.delta_speed=swarm_tools::clip(0-agent_state.speed, -constraints.max_delta_speed, constraints.max_delta_speed);
        return -1; // Indicates the fan was completely blocked 
    }
    // std::cout << "Requested command: " << command.delta_heading*180/swarm_tools::PI<<std::endl;

    // restrict command using differential constraints
    double delta_heading = swarm_tools::clip(command.delta_heading,
                                             -max_angle_rad,
                                             max_angle_rad);

    if (is_command_safe(delta_heading, safe_intervals)){
        return 0; // Indicates no change was made.
    }

    // Get largest interval
    long nearest_interval_index = nearest_safe_interval(safe_intervals, delta_heading);
    swarm_tools::AngleInterval nearest_interval = safe_intervals[nearest_interval_index];

    double l_theta_rad = nearest_interval.l_theta_rad;
    double r_theta_rad = nearest_interval.r_theta_rad;
    // std::cout << "Largest Safe Interval: (" <<l_theta_rad*180/swarm_tools::PI << ", " << r_theta_rad*180/swarm_tools::PI << ")" << std::endl;

    // Get the weighted mid point of the interval.
    double l_theta_dist = std::abs(swarm_tools::radnorm(l_theta_rad-delta_heading));
    double r_theta_dist = std::abs(swarm_tools::radnorm(r_theta_rad-delta_heading));
    bool go_left;
    if ( l_theta_dist < r_theta_dist){
        go_left = true;
    }else{
        go_left = false;
    }


    if(go_left){
        command.delta_heading = aggression*l_theta_rad + (1-aggression)*r_theta_rad;
    }else{
        command.delta_heading = aggression*r_theta_rad + (1-aggression)*l_theta_rad;
    }
//    command.delta_heading /= 0.1;
    // std::cout << "Weighted heading command: "<< command.delta_heading*180/swarm_tools::PI; 

    // restrict command using differential constraint
//    if(command.delta_speed*0.1 + agent_state.speed>constraints.max_speed){
//        command.delta_speed = (constraints.max_speed-agent_state.speed);
//    }
    command.delta_heading = swarm_tools::clip(command.delta_heading, -max_angle_rad, max_angle_rad);
    return 1;
}

bool collision_avoidance::is_command_safe(
    const double &delta_heading,
    const std::vector<swarm_tools::AngleInterval>& safe_intervals
){
    for (const swarm_tools::AngleInterval interval : safe_intervals){
//        double l_theta = interval.l_theta_rad;
//        double r_theta = interval.r_theta_rad;
        // std::cout << "(" <<l_theta*180/swarm_tools::PI << ", " << r_theta*180/swarm_tools::PI << ")"<< std::endl;
        if (interval.contains(delta_heading)){
            return true;
        }
    }
    return false;
}

int collision_avoidance::largest_safe_interval(
    const std::vector<swarm_tools::AngleInterval>& safe_intervals
){
    int max_index;
    double max_diff = -1;
    for (auto ip=safe_intervals.begin();ip != safe_intervals.end(); ip++){
        double diff = std::abs(ip->l_theta_rad-ip->r_theta_rad);
        std::cout << "Looking for largest interval(" <<ip->l_theta_rad*180/swarm_tools::PI << ", " << ip->r_theta_rad*180/swarm_tools::PI << ")"<< std::endl;
        if (diff>=max_diff){
            max_index = distance(safe_intervals.begin(), ip);
            max_diff = diff;
        }
    }
    return max_index;
}

long collision_avoidance::nearest_safe_interval(
    const std::vector<swarm_tools::AngleInterval>& safe_intervals,
    const double &delta_heading
){
    long min_index=-1;
    double min_diff = 1e100;
    double diff_left;
    double diff_right;
    double diff;
    for (auto ip=safe_intervals.begin();ip != safe_intervals.end(); ip++){
        diff_left = std::abs(ip->l_theta_rad-delta_heading);
        diff_right = std::abs(ip->r_theta_rad-delta_heading);
        diff = std::min(diff_left, diff_right);
        // std::cout << "Looking for nearest interval(" <<ip->l_theta_rad*180/swarm_tools::PI << ", " << ip->r_theta_rad*180/swarm_tools::PI << ")"<< std::endl;
        if (diff<=min_diff){
            min_index = distance(safe_intervals.begin(), ip);
            min_diff = diff;
        }
    }
    return min_index;
}

bool collision_avoidance::collision_check(
    const agent::AgentState &agent_state,
    const swarm_tools::Point2D &left,
    const swarm_tools::Point2D &right,
    double max_distance,
    double max_angle_rad,
    double &l_theta_rad,
    double &r_theta_rad
){

    swarm_tools::Point2D agent_position = agent_state.get_position();
    swarm_tools::Point2D mid_point = swarm_tools::mid_point(left, right);
    
    // Get distance to mid point of object
    double distance = swarm_tools::euclidean_distance(agent_position, mid_point);

    // Get the relative angle to the edge points of object
    l_theta_rad = swarm_tools::relative_angle_between_points(agent_position, left, agent_state.heading);
    r_theta_rad = swarm_tools::relative_angle_between_points(agent_position, right, agent_state.heading);

    bool is_in_fan = in_fan(distance, l_theta_rad, r_theta_rad, max_distance, max_angle_rad);
    // std::cout << "Collision Check: (" <<l_theta_rad*180/swarm_tools::PI << ", " << r_theta_rad*180/swarm_tools::PI << ") Distance: "<< distance << " Flag: "<< is_in_fan << std::endl;

    if (is_in_fan){
        l_theta_rad = swarm_tools::clip(l_theta_rad, -max_angle_rad, max_angle_rad);
        r_theta_rad = swarm_tools::clip(r_theta_rad, -max_angle_rad, max_angle_rad);
    }
    // std::cout <<"Fan: " << is_in_fan<<std::endl;

    return is_in_fan;
}//collision_check

bool collision_avoidance::in_fan(
    double distance,
    double l_theta_rad,
    double r_theta_rad,
    double max_distance,
    double max_angle_rad
){
    if (l_theta_rad>= max_angle_rad && r_theta_rad <= -max_angle_rad && distance<max_distance){return true;}
    // Check if angles are in range
    bool left_in_range = -max_angle_rad <= l_theta_rad && l_theta_rad <= max_angle_rad;
    bool right_in_range = -max_angle_rad <= r_theta_rad && r_theta_rad <= max_angle_rad;

    if (!(left_in_range || right_in_range) || distance>max_distance){
        return false; // Angles are out of range
    }
 
    if (l_theta_rad >= 0 and r_theta_rad<=0){
        return true;
    }
    else{
        return l_theta_rad >= r_theta_rad;
    }
}//in_fan


int collision_avoidance::get_safe_intervals(
    const agent::AgentState& agent_state,
    const std::vector<swarm_tools::PointInterval>& edge_intervals,
    double max_distance,
    double max_angle_rad,
    std::vector<swarm_tools::AngleInterval>& safe_intervals
){
    std::vector<swarm_tools::AngleInterval> occupied_intervals;
    // std::cout << "Size: " << edge_intervals.size() << std::endl;
    for (swarm_tools::PointInterval point_interval : edge_intervals){
        swarm_tools::Point2D left_point = point_interval.left_point;
        swarm_tools::Point2D right_point = point_interval.right_point;

        double l_theta;
        double r_theta;

        bool flag = collision_check(agent_state,
                                   left_point,
                                   right_point,
                                   max_distance,
                                   max_angle_rad,
                                   l_theta,
                                   r_theta);
        if (!flag){
            continue;
        }else{
            const swarm_tools::AngleInterval interval = {l_theta, r_theta};
            occupied_intervals.push_back(interval);
        }
    }
    std::sort(occupied_intervals.begin(), occupied_intervals.end(), swarm_tools::greater_ai);
    swarm_tools::AngleInterval safe_interval;
    double right_theta_rad = max_angle_rad;
    for ( auto ip = occupied_intervals.begin(); ip <= occupied_intervals.end(); ip++){
        if (ip==occupied_intervals.end()){
            if(right_theta_rad > -max_angle_rad){
                safe_interval = {right_theta_rad, -max_angle_rad};
                safe_intervals.push_back(safe_interval);
                return 1;
            }
        }else{
            if(right_theta_rad <= ip->l_theta_rad){
                if (right_theta_rad>= ip->r_theta_rad){
                    right_theta_rad = ip->r_theta_rad;
                }
            }else{
                safe_interval = swarm_tools::AngleInterval{right_theta_rad, ip->l_theta_rad};
                safe_intervals.push_back(safe_interval);
                right_theta_rad = ip->r_theta_rad;
                // return safe_intervals.size();
            }
        }
    }

    return -1;
}
