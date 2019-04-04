#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <assert.h>
#include "ros_swarm_tools.h"
#include "motion_goal_control.h"
#include "swarm_threat_detection/batchIntruderCommands.h"

namespace swarm_control{
    bool usv_delay_motion_goal(
        int usv_id,
        int intruder_id,
        const agent::USVSwarm &swarm,
        agent::MotionGoal& motion_goal
    ){
        int delay_position = swarm.get_delay_position(usv_id, intruder_id);
        auto usv = swarm.get_usv_estimate_by_id(usv_id);
        auto intruder = swarm.get_intruder_estimate_by_id(intruder_id);
        auto asset = swarm.get_asset_estimate();

        auto usv_state = usv.get_state();
        auto usv_constraints = usv.get_constraints();

        auto intruder_state = intruder.get_state();
        auto intruder_constraints = intruder.get_constraints();

        auto asset_state = asset.get_state();
        double capture_radius = 10;
        double w_delayed = 0.5;

        motion_goal.speed = std::max(intruder_state.speed -delay_position*w_delayed, 0.0);
        motion_goal.heading_rad = intruder_state.heading;
        swarm_tools::Point2D usv_position = usv_state.get_position();
        swarm_tools::Point2D intruder_position = intruder_state.get_position();
        intruder_position.x += capture_radius*std::cos(intruder_state.heading);
        intruder_position.y += capture_radius*std::sin(intruder_state.heading);
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

        if (det<0){
            // Not solutions
            motion_goal.x = asset_position.x;
            motion_goal.y = asset_position.y;
            return false;
        } 
        else
        {
            double alpha_minus;
            double alpha_plus;
            double min_alpha = std::min(capture_radius/o_norm, 1.0);

            if (A==0){
                alpha_minus = C/B;
                alpha_plus = 10;
            }else{
                alpha_minus = (-B-sqrt(det))/(2*A);
                alpha_plus = (-B+sqrt(det))/(2*A);
            }
            swarm_tools::Point2D* intercept_1_ptr = nullptr;
            swarm_tools::Point2D* intercept_2_ptr = nullptr;
            swarm_tools::Point2D intercept_1{};
            swarm_tools::Point2D intercept_2{};

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
                    }
                else{
                        double dist_1 = swarm_tools::euclidean_distance(intercept_1, usv_position); 
                        double dist_2 = swarm_tools::euclidean_distance(intercept_2, usv_position); 

                        if (dist_1 <=dist_2){
                            motion_goal.x = intercept_1_ptr->x;
                            motion_goal.y = intercept_1_ptr->y;
                        }
                        else{
                            motion_goal.x = intercept_2_ptr->x;
                            motion_goal.y = intercept_2_ptr->y;
                        }

                    }
            }
            else if (intercept_1_ptr==nullptr && intercept_2_ptr==nullptr){
                alpha_minus = C/B;
                if (0<=alpha_minus && alpha_minus <= 1){
                    alpha_minus = std::max(alpha_minus, min_alpha);
                    intercept_1 = (intruder_position + (asset_position-intruder_position)*alpha_minus); 
                    motion_goal.x = intercept_1.x;
                    motion_goal.y = intercept_1.y;
                }else{
                    intercept_1 = (intruder_position + (asset_position-intruder_position)*min_alpha);
                    motion_goal.x = intercept_1.x;
                    motion_goal.y = intercept_1.y;
                }
                return false;
            }
            
            else if (intercept_1_ptr==nullptr){
                motion_goal.x = intercept_2_ptr->x;
                motion_goal.y = intercept_2_ptr->y;
                return true;
            }

            else if (intercept_2_ptr==nullptr){
                motion_goal.x = intercept_1_ptr->x;
                motion_goal.y = intercept_1_ptr->y;
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

    bool usv_guard_motion_goal(int num_of_usvs,
                               int guard_id,
                               double asset_radius,
                               const agent::AgentState asset_state,
                               agent::MotionGoal& motion_goal
    ){
        double angle = (guard_id/(double) num_of_usvs) * 2*swarm_tools::PI;
        swarm_tools::Point2D asset_location = asset_state.get_position();
        swarm_tools::Point2D offset = {asset_radius*std::cos(angle), asset_radius*std::sin(angle)};
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
    bool observed_intruder_motion_goal(
            int intruder_id,
            const agent::USVSwarm &swarm,
            agent::MotionGoal &motion_goal){
        auto intruder = swarm.get_intruder_estimate_by_id(intruder_id);
        auto asset = swarm.get_asset_estimate();
        return intruder_motion_goal(asset.get_state(), motion_goal);
    }

    double get_speed_goal(double distance_to_mg,
                          double agent_max_speed,
                          double near_mg_dist,
                          double accepted_mg_dist
                          ){
        double speed_goal;
        if (distance_to_mg> near_mg_dist){
            speed_goal = agent_max_speed;
        }
        else if(distance_to_mg > accepted_mg_dist){
            double alpha = distance_to_mg/near_mg_dist;
            speed_goal = alpha*agent_max_speed;
        }
        else{
            speed_goal = 0;
        }
        return speed_goal;
    }

    void get_left_and_right_turns(double heading_goal,
                                  double current_heading,
                                  double &left_turn,
                                  double &right_turn){

        if (heading_goal<0) heading_goal += 2*swarm_tools::PI;
        left_turn = heading_goal-current_heading;

        if (left_turn<0) left_turn+= 2*swarm_tools::PI;
        right_turn = -(2*swarm_tools::PI-left_turn);
    }
   double get_smallest_delta_heading(double left_turn,
                                      double right_turn,
                                      double current_heading,
                                      double max_delta_heading){
        double delta_heading;
        if (std::abs(left_turn)<=std::abs(right_turn)) {
            delta_heading = left_turn;
            }
        else {
            delta_heading = right_turn;
            }

        delta_heading = swarm_tools::clip(delta_heading,
                                          -max_delta_heading,
                                          max_delta_heading);
        return delta_heading;
    }
    double get_smallest_delta_heading(double heading_goal,
                                      double current_heading,
                                      double max_delta_heading){
        double left_turn;
        double right_turn;
        get_smallest_delta_heading(left_turn, right_turn, current_heading, max_delta_heading);
        double delta_heading;
        if (std::abs(left_turn)<=std::abs(right_turn)) {
            delta_heading = left_turn;
            }
        else {
            delta_heading = right_turn;
            }

        delta_heading = swarm_tools::clip(delta_heading,
                                          -max_delta_heading,
                                          max_delta_heading);
        return delta_heading;
    }

    double get_largest_delta_heading(double left_turn,
                                      double right_turn,
                                      double current_heading,
                                      double max_delta_heading){
        double delta_heading;
        if (std::abs(left_turn)>std::abs(right_turn)) {
            delta_heading = left_turn;
            }
        else {
            delta_heading = right_turn;
            }

        delta_heading = swarm_tools::clip(delta_heading,
                                          -max_delta_heading,
                                          max_delta_heading);
        return delta_heading;
    }
    double get_delta_heading_towards_asset(double left_turn,
                                           double right_turn,
                                           double current_heading,
                                           double angle_to_asset,
                                           double max_delta_heading){
        double delta_heading;
        double heading_towards_asset = angle_to_asset-current_heading;
        if(std::abs(left_turn -heading_towards_asset) < std::abs(right_turn-heading_towards_asset)){
            delta_heading = left_turn;
        }else{
            delta_heading = right_turn;
        }

        delta_heading = swarm_tools::clip(delta_heading,
                                          -max_delta_heading,
                                          max_delta_heading);
        return delta_heading;
    }
    double get_delta_speed(double speed_goal,
                           double current_speed,
                           double max_delta_speed){
        double delta_speed = (speed_goal-current_speed);
        delta_speed = swarm_tools::clip(delta_speed,
                                        -max_delta_speed,
                                        max_delta_speed);
        return delta_speed;
    }

    template<typename T> bool get_batch_intruder_commands_from_model(
            const std::vector<T> &intruders,
            std::map<int, agent::AgentCommand> &commands,
            ros::ServiceClient &client){

        swarm_threat_detection::batchIntruderCommands srv;
        get_batch_intruder_previous_states_msg(intruders,
                srv.request.batch_previous_states);
        commands.clear();

        if(client.call(srv)){
            ROS_INFO("Batch Intruder Model Query Successful!");
            for (const auto &command_msg : srv.response.batch_intruder_commands){
                commands[command_msg.sim_id] = extract_from_command_msg(command_msg);
            }
            return true;

        }else {
            ROS_INFO("Batch Intruder Model Query Failed!");
            ROS_ERROR("Batch Intruder Model Query Failed!");
            return false;
        }
    }
    template bool get_batch_intruder_commands_from_model<agent::ObservedIntruderAgent>(
            const std::vector<agent::ObservedIntruderAgent> &intruders,
            std::map<int, agent::AgentCommand> &commands,
            ros::ServiceClient &client);
    template bool get_batch_intruder_commands_from_model<agent::IntruderAgent>(
            const std::vector<agent::IntruderAgent> &intruders,
            std::map<int, agent::AgentCommand> &commands,
            ros::ServiceClient &client);

    bool get_intruder_command_from_motion_goal(const agent::IntruderAgent& intruder,
            const agent::MotionGoal &motion_goal,
            agent::AgentCommand &command) {

        swarm_tools::Point2D intruder_position = intruder.get_position();
        swarm_tools::Point2D mg_position = motion_goal.get_position();
        double heading_goal = swarm_tools::absolute_angle_between_points(intruder_position,
                                                                         mg_position);
        double distance_to_mg = swarm_tools::euclidean_distance(intruder_position,
                                                                mg_position);
        double near_mg_dist=25;
        double accepted_mg_dist=10;
        double speed_goal = get_speed_goal(distance_to_mg,
                                           intruder.get_max_speed(),
                                           near_mg_dist,
                                           accepted_mg_dist);

        double left_turn;
        double right_turn;
        double current_heading = intruder.get_heading();

        get_left_and_right_turns(heading_goal, current_heading, left_turn, right_turn);
        double delta_heading;
        if(intruder.evade()){
            delta_heading = get_largest_delta_heading(left_turn,
                                                              right_turn,
                                                              current_heading,
                                                              intruder.get_max_delta_heading());

        }else{
            delta_heading = get_smallest_delta_heading(left_turn,
                                                       right_turn,
                                                       current_heading,
                                                       intruder.get_max_delta_heading());
        }
        double delta_speed = get_delta_speed(speed_goal,
                                             intruder.get_speed(),
                                             intruder.get_max_delta_speed());
        command.delta_speed=delta_speed;
        command.delta_heading=delta_heading;
    }

    bool get_observed_intruder_command_from_motion_goal(const agent::ObservedIntruderAgent& intruder,
                                                        const agent::MotionGoal &motion_goal,
                                                        agent::AgentCommand &command) {

        swarm_tools::Point2D intruder_position = intruder.get_position();
        swarm_tools::Point2D mg_position = motion_goal.get_position();
        double heading_goal = swarm_tools::absolute_angle_between_points(intruder_position,
                                                                         mg_position);
        double distance_to_mg = swarm_tools::euclidean_distance(intruder_position,
                                                                mg_position);
        double near_mg_dist=25;
        double accepted_mg_dist=10;
        double speed_goal = get_speed_goal(distance_to_mg,
                                           intruder.get_max_speed(),
                                           near_mg_dist,
                                           accepted_mg_dist);

        double left_turn;
        double right_turn;
        double current_heading = intruder.get_heading();

        get_left_and_right_turns(heading_goal, current_heading, left_turn, right_turn);
        double delta_heading = get_smallest_delta_heading(left_turn,
                                                          right_turn,
                                                          current_heading,
                                                          intruder.get_max_delta_heading());
        double delta_speed = get_delta_speed(speed_goal,
                                             intruder.get_speed(),
                                             intruder.get_max_delta_speed());
        command.delta_speed=delta_speed;
        command.delta_heading=delta_heading;
    }

    bool get_usv_command_from_motion_goal(
        int usv_id,
        const agent::USVSwarm& swarm,
        const agent::MotionGoal& motion_goal,
        agent::AgentCommand& command){

        agent::USVAgent usv = swarm.get_usv_estimate_by_id(usv_id);
        swarm_tools::Point2D usv_position = usv.get_position();
        swarm_tools::Point2D mg_position = motion_goal.get_position();
        double heading_goal = swarm_tools::absolute_angle_between_points(usv_position,
                                                                          mg_position);
        double distance_to_mg = swarm_tools::euclidean_distance(usv_position,
                                                          mg_position);
        double speed_goal;

        if (usv.has_delay_task()){
            double near_mg_dist=20;
            double accepted_mg_dist=5;
            speed_goal = get_speed_goal(distance_to_mg, usv.get_max_speed(), near_mg_dist, accepted_mg_dist);
        }else{
            double near_mg_dist=25;
            double accepted_mg_dist=5;
            speed_goal = get_speed_goal(distance_to_mg, usv.get_max_speed(), near_mg_dist, accepted_mg_dist);
        }

        double left_turn;
        double right_turn;
        double current_heading = usv.get_heading();
        double angle_to_asset = swarm_tools::absolute_angle_between_points(usv_position,
                                                                           swarm.get_asset_estimate().get_position());
        get_left_and_right_turns(heading_goal, current_heading, left_turn, right_turn);
        double delta_heading = get_smallest_delta_heading(left_turn,
                                                          right_turn,
                                                          current_heading,
                                                          usv.get_max_delta_heading());

        double delta_speed = get_delta_speed(speed_goal,
                                             usv.get_speed(),
                                             usv.get_max_delta_speed());

        // ROS_INFO("%f, %f, %f, %f, %f, %f", delta_speed, delta_heading, speed_goal, heading_goal, usv.get_max_speed(), usv.get_max_delta_heading());
        command.delta_speed=delta_speed;
        command.delta_heading=delta_heading;
        }

    double weighted_motion_goal(
        const std::vector<agent::MotionGoal>& motion_goals,
        const std::vector<double>& weights,
        agent::MotionGoal& weighted_motion_goal
    ){
        double weighted_x=0;
        double weighted_y=0;
        double weighted_heading=0;
        double w_sum = 1e-10;

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
        return w_sum;
    }
    double get_motion_goal_from_assignment(int usv_id,
                                         const agent::USVSwarm &swarm,
                                         agent::MotionGoal &motion_goal){
        std::vector<agent::MotionGoal> motion_goals;
        std::vector<double> weights;
        agent::MotionGoal mg;
        agent::USVAgent usv = swarm.get_usv_estimate_by_id(usv_id);
        agent::AgentAssignment assignment = usv.get_current_assignment();
        agent::AgentState asset_state = swarm.get_asset_estimate().get_state();

        int num_usvs = swarm.get_num_usvs();
        int guard_radius = 100;
        double w_guard = agent::get_guard_weight();
        int guard_id;
        int intruder_id;
        double dist_to_asset;
        double p_threat;
        double weight;
        agent::ObservedIntruderAgent intruder;
        if(assignment.empty()) {
            ROS_ERROR("Task assignment for usv %d is empty", usv_id);
            motion_goal.x = 100;
            motion_goal.y = 100;
            return 0.1;
        }
        for(const auto &task : assignment){
            if(task.task_idx==-1) {
                ROS_ERROR("Empty task in motion goal prediction for usv %d", usv_id);
                continue;
            };
            switch (task.task_type){
                case agent::TaskType::Delay:
                    intruder_id = task.task_idx;
                    swarm_control::usv_delay_motion_goal(usv_id, intruder_id, swarm, motion_goal);
                    return 1.0;
                case agent::TaskType::Guard:
                    guard_id=task.task_idx;
                    asset_state=swarm.get_asset_estimate().get_state();
                    swarm_control::usv_guard_motion_goal(num_usvs, guard_id, guard_radius, asset_state, mg);
                    motion_goals.push_back(mg);
                    weights.push_back(w_guard);
                    break;
                case agent::TaskType::Observe:
                    intruder_id=task.task_idx;
                    swarm_control::usv_observe_motion_goal(usv_id, intruder_id, swarm, mg);
                    motion_goals.push_back(mg);
                    intruder = swarm.get_intruder_estimate_by_id(intruder_id);
//                    usv = swarm.get_usv_estimate_by_id(usv_id);
                    weight = agent::get_observation_weight(intruder, swarm.get_asset_estimate());
                    weights.push_back(weight);
                    // ROS_INFO("Distance %f, Probability %f, Weight %f", dist_to_asset, p_threat, weight);
                    break;
                default:
                    break;
            }
        }
        if(motion_goals.empty()) {
            ROS_ERROR("Motion goals for usv %d is empty", usv_id);
            motion_goal.x=100;
            motion_goal.y=100;
            return 0.1;
        }
        double total_weight = weighted_motion_goal(motion_goals, weights, motion_goal);
        return 1-(w_guard/total_weight);
    }
    bool usv_observe_motion_goal(int usv_id,
                                 int intruder_id,
                                 const agent::USVSwarm &swarm,
                                 agent::MotionGoal &motion_goal){
        auto intruder_state = swarm.get_intruder_estimate_by_id(intruder_id).get_state();
        motion_goal.x=intruder_state.x;
        motion_goal.y=intruder_state.y;
        motion_goal.speed=intruder_state.speed;
        motion_goal.heading_rad=intruder_state.heading;
    }
}