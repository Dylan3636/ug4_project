#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <assert.h>
#include "motion_goal_control.h"

namespace swarm_control{
    bool usv_delay_motion_goal(const agent::USVAgent &usv,
                               const agent::IntruderAgent &intruder,
                               const agent::AssetAgent &asset,
                               agent::MotionGoal &motion_goal){

        return usv_delay_motion_goal(usv.get_state(),
                                     usv.get_constraints(),
                                     intruder.get_state(),
                                     intruder.get_constraints(),
                                     asset.get_state(),
                                     motion_goal);

    }
    bool usv_delay_motion_goal(int usv_id,
                               int intruder_id,
                               agent::USVSwarm){
        
    }
    bool usv_delay_motion_goal(
        const agent::AgentState& usv_state,
        const agent::AgentConstraints& usv_constraints,
        const agent::AgentState& intruder_state,
        const agent::AgentConstraints& intruder_constraints,
        const agent::AgentState& asset_state,
        agent::MotionGoal& motion_goal
    ){
        // ROS_INFO("USV DELAY: ");
        motion_goal.speed = intruder_state.speed;
        motion_goal.heading_rad = intruder_state.heading;
        swarm_tools::Point2D usv_position = usv_state.get_position();
        swarm_tools::Point2D intruder_position = intruder_state.get_position();
        intruder_position.x += 20*std::cos(intruder_state.heading);
        intruder_position.y += 20*std::sin(intruder_state.heading);
        swarm_tools::Point2D asset_position = asset_state.get_position();

        double usv_max_speed = usv_constraints.max_speed;// Remember this
        double intruder_max_speed = intruder_constraints.max_speed;

        swarm_tools::Vector2D d = {usv_position, intruder_position};
        swarm_tools::Vector2D o = {intruder_position, asset_position};

        double o_norm = o.norm();
        double d_norm = d.norm();

        double A = o_norm*o_norm * (1 + -(usv_max_speed*usv_max_speed/(intruder_max_speed*intruder_max_speed)));
        double B = 2*d.dot(o); //Ax^2 -Bx + C
        double C = d_norm*d_norm;

        double det = B*B -4*A*C;
        // ROS_INFO("A: %f, B: %f, C: %f", A, B, C);

        if (det<0){
            // Not possible 
            motion_goal.x = asset_position.x;
            motion_goal.y = asset_position.y;
            // ROS_INFO("ALPHA: NO SOLUTIONS FOUND");
            return false;
        } 
        else
        {
            double alpha_minus;
            double alpha_plus;
            double capture_radius = 10;
            double min_alpha = std::min(capture_radius/o_norm, 1.0);

            if (A==0){
                alpha_minus = C/B;
                alpha_plus = 10;
            }else{
                alpha_minus = (-B-sqrt(det))/(2*A);
                alpha_plus = (-B+sqrt(det))/(2*A);
            }
            // ROS_INFO("ALPHA-: %f", alpha_minus);
            // ROS_INFO("ALPHA+: %f", alpha_plus);
            swarm_tools::Point2D* intercept_1_ptr = nullptr;
            swarm_tools::Point2D* intercept_2_ptr = nullptr;
            swarm_tools::Point2D intercept_1;
            swarm_tools::Point2D intercept_2;

            if (0<=alpha_minus && alpha_minus <= 1){
                alpha_minus = std::max(alpha_minus, min_alpha);
                intercept_1 = (intruder_position + (asset_position-intruder_position)*alpha_minus); 
                intercept_1_ptr = &intercept_1;
            }
            if (0<=alpha_plus && alpha_plus <= 1){
                alpha_plus = std::max(alpha_plus, min_alpha);
                intercept_2 = (intruder_position + (asset_position-intruder_position)*alpha_plus); 
                intercept_2_ptr = &intercept_2;
            }

            if(intercept_1_ptr!=nullptr && intercept_2_ptr!=nullptr){
                if (det==0){
                        motion_goal.x = intercept_1_ptr->x;
                        motion_goal.y = intercept_1_ptr->y;
                        // ROS_INFO("USING ALPHA-: %f", alpha_minus);
                    }
                else{
                        // swarm_tools::Point2D intercept_1 = *intercept_1_ptr;
                        // swarm_tools::Point2D intercept_2 = *intercept_2_ptr;
                        double dist_1 = swarm_tools::euclidean_distance(intercept_1, usv_position); 
                        double dist_2 = swarm_tools::euclidean_distance(intercept_2, usv_position); 

                        if (dist_1 <=0){
                            motion_goal.x = intercept_1_ptr->x;
                            motion_goal.y = intercept_1_ptr->y;
                            // ROS_INFO("USING ALPHA-: %f", alpha_minus);
                        }
                        else{
                            // ROS_INFO("USING ALPHA+: %f", alpha_plus);
                            motion_goal.x = intercept_2_ptr->x;
                            motion_goal.y = intercept_2_ptr->y;
                        }

                    }
            }
            else if (intercept_1_ptr==nullptr && intercept_2_ptr==nullptr){
                alpha_minus = C/B;
                if (0<=alpha_minus && alpha_minus <= 1){
                    alpha_minus = std::max(alpha_minus, min_alpha);
                    // ROS_INFO("USING ALPHA-: %f", alpha_minus);
                    intercept_1 = (intruder_position + (asset_position-intruder_position)*alpha_minus); 
                    motion_goal.x = intercept_1.x;
                    motion_goal.y = intercept_1.y;
                }else{
                    intercept_1 = (intruder_position + (asset_position-intruder_position)*0.1); 
                    motion_goal.x = intercept_1.x;
                    motion_goal.y = intercept_1.y;
                }
                return false;
            }
            
            else if (intercept_1_ptr==nullptr){
                motion_goal.x = intercept_2_ptr->x;
                motion_goal.y = intercept_2_ptr->y;
                // ROS_INFO("USING ALPHA+: %f", alpha_plus);
                return true;
            }

            else if (intercept_2_ptr==nullptr){
                motion_goal.x = intercept_1_ptr->x;
                motion_goal.y = intercept_1_ptr->y;
                // ROS_INFO("USING ALPHA-: %f", alpha_minus);
                return true;
            }

            

            
            return true;
        }
        return true;
    } // usv_delay_motion_goal

    bool usv_block_motion_goal(
        const agent::AgentState& usv_state,
        const agent::AgentConstraints& usv_constraints,
        const double &capture_radius,
        const agent::AgentState& intruder_state,
        const agent::AgentConstraints& intruder_constraints,
        const double &intruder_radius,
        const agent::AgentState& asset_state,
        agent::MotionGoal& motion_goal
    ){
        motion_goal.speed = intruder_state.speed;
        motion_goal.heading_rad = intruder_state.heading;
        swarm_tools::Point2D usv_position = usv_state.get_position();
        swarm_tools::Point2D intruder_position = intruder_state.get_position();
        swarm_tools::Point2D asset_position = asset_state.get_position();

        double usv_max_speed = intruder_constraints.max_speed;// Remember this
        double intruder_max_speed = intruder_constraints.max_speed;

        swarm_tools::Vector2D d = {usv_position, intruder_position};
        swarm_tools::Vector2D o = {intruder_position, asset_position};


        double distance_to_intruder = d.norm();
        double theta = std::asin(intruder_radius/distance_to_intruder);
        double blocking_distance = capture_radius/cos(theta);
        double alpha = blocking_distance/distance_to_intruder;
        double max_alpha = std::max((distance_to_intruder-capture_radius + 0.0)/distance_to_intruder, 0.0);

        alpha = std::min(max_alpha, alpha);
        auto intercept= (intruder_position + (asset_position-intruder_position)*0.2); 
        motion_goal.x = intercept.x;
        motion_goal.y = intercept.y;
        // ROS_INFO("USING ALPHA: %f", alpha);
        return true;
    }

    bool usv_guard_motion_goal(const int& num_of_usvs,
                               const int& usv_assignment,
                               const double& radius,
                               const agent::AgentState asset_state,
                               agent::MotionGoal& motion_goal
    ){
        double angle = (usv_assignment/(double) num_of_usvs) * 2*swarm_tools::PI;
        swarm_tools::Point2D asset_location = asset_state.get_position();
        swarm_tools::Point2D offset = {radius*std::cos(angle), radius*std::sin(angle)};
        swarm_tools::Point2D guard_position = asset_location+offset;

        motion_goal.x=guard_position.x; 
        motion_goal.y=guard_position.y; 
        motion_goal.heading_rad = angle;
        return true;
    }
    bool intruder_motion_goal(const agent::AgentState &asset,
                              agent::MotionGoal &motion_goal){

        motion_goal = {asset.x, asset.y};
    }

    bool get_command_from_motion_goal(
        const agent::AgentState& agent_state,
        const agent::AgentConstraints& agent_constraints,
        const agent::MotionGoal& motion_goal,
        agent::AgentCommand& command
    ){
        double dist_param = 5;
        double slow_param = 10; // Distance to MG in which the agent starts slowing
        double angle_between = swarm_tools::absolute_angle_between_points(agent_state.get_position(),
                                                                          motion_goal.get_position());
        double distance = swarm_tools::euclidean_distance(agent_state.get_position(),
                                                          motion_goal.get_position());
        double heading_goal;
        double speed_goal; 

        heading_goal = angle_between;
        if (distance>slow_param){
            speed_goal = agent_constraints.max_speed;
        }
        else if(distance>dist_param){
            double alpha = distance/slow_param;
            speed_goal = alpha*agent_constraints.max_speed;
        }
        else{
            speed_goal = 0;
        }
        if (heading_goal<0) heading_goal += 2*swarm_tools::PI;
        double left_turn = heading_goal-agent_state.heading;

        if (left_turn<0) left_turn+= 2*swarm_tools::PI;
        double right_turn = -(2*swarm_tools::PI-left_turn);


        double delta_heading;
        if (std::abs(left_turn)<=std::abs(right_turn)) {delta_heading = left_turn;} else {delta_heading = right_turn;}

        delta_heading = swarm_tools::clip(delta_heading,
                                        -agent_constraints.max_delta_heading,
                                        agent_constraints.max_delta_heading);
        
        double delta_speed = speed_goal-agent_state.speed;
        delta_speed = swarm_tools::clip(delta_speed,
                                        -agent_constraints.max_delta_speed,
                                        agent_constraints.max_delta_speed);

        command.delta_speed=delta_speed;
        command.delta_heading=delta_heading;
        return true;
    }

    bool weighted_motion_goal(
        const std::vector<agent::MotionGoal>& motion_goals,
        const std::vector<double>& weights,
        agent::MotionGoal& weighted_motion_goal
    ){
        double weighted_x=0;
        double weighted_y=0;
        double weighted_heading=0;
        double w_sum = 0;

        for (int i = 0; i<weights.size(); i++){
            double w = weights[i];
            agent::MotionGoal mg = motion_goals[i];
            weighted_x += w*mg.x;
            weighted_y +=  w*mg.y;
            weighted_heading +=  w*mg.heading_rad;
            w_sum += w;
        }

        weighted_x /= w_sum;
        weighted_y /= w_sum;
        weighted_heading /= w_sum;
        weighted_motion_goal.x=weighted_x;
        weighted_motion_goal.y=weighted_y;
        weighted_motion_goal.heading_rad=weighted_heading;
        return true;
    }
    bool get_motion_goals_from_assignment(int usv_id,
                                          const agent::USVSwarm &swarm,
                                          const agent::AssetAgent &asset,
                                          agent::MotionGoal &delay_motion_goal,
                                          agent::MotionGoal &guard_motion_goal,
                                          agent::MotionGoal &motion_goal
                                          ){
        agent::USVAgent usv = swarm.get_usv_estimate_by_id(usv_id);
        int num_usvs = swarm.get_num_usvs();
        agent::AgentAssignment assignment = usv.get_current_assignment();
        if (assignment.delay_assignment_idx != -1 && assignment.guard_assignment_idx != -1){
            int guard_index = assignment.guard_assignment_idx;
            int intruder_id = assignment.delay_assignment_idx;
            // ROS_INFO("Assignment: Delay index: %d Guard index: %d", intruder_id, guard_index);
            agent::IntruderAgent intruder = swarm.get_intruder_estimate_by_id(intruder_id);
            
            swarm_control::usv_delay_motion_goal(usv,
                                                 intruder,
                                                 asset,
                                                 delay_motion_goal);
            
            swarm_control::usv_guard_motion_goal(num_usvs,
                                                 guard_index,
                                                 100,
                                                 asset.get_state(),
                                                 guard_motion_goal
                                                 );
            std::vector<agent::MotionGoal> mgs = {delay_motion_goal, guard_motion_goal};
            double dist = swarm_tools::euclidean_distance(intruder.get_position(), asset.get_position());
            double alpha = dist/3000;
            if (dist>3000){
                alpha = 0.5;
            }
            std::vector<double> weights = {1, 0};
            swarm_control::weighted_motion_goal(mgs,
                                                weights,
                                                motion_goal);
            
        }else if(assignment.guard_assignment_idx!=-1){
            int guard_index = assignment.guard_assignment_idx;
            // ROS_INFO("Assignment: Delay index: NULL Guard index %d", guard_index);
            swarm_control::usv_guard_motion_goal(num_usvs,
                                                 guard_index,
                                                 100,
                                                 asset.get_state(),
                                                 motion_goal
                                                 );
        
        
        } else if(assignment.delay_assignment_idx != -1){
            int intruder_id = assignment.delay_assignment_idx;
            // ROS_INFO("Assignment: Delay index: %d Guard index: NULL", intruder_id);
            agent::IntruderAgent intruder = swarm.get_intruder_estimate_by_id(intruder_id);
            swarm_control::usv_delay_motion_goal(usv,
                                                 intruder,
                                                 asset,
                                                 motion_goal);
        }
           
        return true;
    }
}