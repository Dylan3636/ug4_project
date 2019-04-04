// #include <string>
// #include <ros/ros.h>
// #include <boost/format.hpp>
// #include <boost/shared_ptr>
#include <mutex>
#include "motion_goal_control.h"
#include "collision_avoidance.h"
#include "ros_swarm_tools.h"
#include "swarm_msgs/simulationMarker.h"
#include "swarm_msgs/threatStatistics.h"
#include "swarm_msgs/initializeSystem.h"
#include "swarm_msgs/resetSystem.h"
#include "task_allocation.h"
#include "swarm_threat_detection/ThreatDetection.h"
#include "swarm_threat_detection/batchIntruderCommands.h"
#include "swarm_task_manager/modelPredictiveSimulation.h"
#include "ctime"

bool initialized = false;
int usv_id;
int intruder_id;
int NUM_USVS;
double threat_sigma;
double non_threat_sigma;
bool vanilla_threat_detection=false;

agent::USVSwarm swarm;

// // Declare ros container
RosContainerPtr ros_container_ptr;
ros::Publisher command_pub;
ros::Publisher marker_pub;
ros::Publisher threat_pub;
ros::Publisher task_allocation_pub;
ros::ServiceClient client;

std::map<int, ros::ServiceClient> usv_sync_service_map;
std::map<int, bool> usv_communication_map;
std::map<int, bool> usv_sync_map;
std::map<int, agent::AgentState> usv_state_map;
std::map<int, agent::AgentConstraints> usv_constraints_map;
std::map<int, agent::CollisionAvoidanceParameters> usv_radar_params_map;
std::map<int, agent::AgentState> intruder_state_map;
std::map<int, agent::AgentConstraints> intruder_constraints_map;
std::map<int, agent::CollisionAvoidanceParameters> intruder_radar_params_map;
agent::AgentState asset_state = agent::AgentState(0,0,0,0,40,0);

std::mutex mtx;
int count_since_shuffle = 0;
bool reset=false;
void publish_markers(const agent::MotionGoal &motion_goal){
    // Motion Goal 
    ROS_INFO("Publishing Motion Goal ([%f], [%f])", motion_goal.x, motion_goal.y);
    swarm_msgs::simulationMarker marker;
    marker.x = motion_goal.x;
    marker.y = motion_goal.y;
    marker.sim_id = 10000 + usv_id; // TODO Fix this.
    marker.colour = "RED";
    marker_pub.publish(marker);
}

double standard_gaussian_liklihood(){
}

double calculate_non_threat_log_likelihood(agent::ObservedIntruderAgent intruder, const std::vector<agent::AgentState> &obstacles){
//    ROS_INFO("Calculating non threat ll");
    agent::AgentState current_state = intruder.previous_states.front();
//    ROS_ERROR("(%f, %f)", current_state.speed, current_state.heading);
    intruder.previous_states.pop_front();
    agent::AgentState previous_state = intruder.previous_states.front();
//    ROS_ERROR("(%f, %f)", previous_state.speed, previous_state.heading);
    intruder.previous_states.pop_front();
    agent::AgentCommand command;
    collision_avoidance::correct_command(intruder, obstacles, command);
    double pred_speed = previous_state.speed + command.delta_speed;
    double pred_heading = previous_state.heading + command.delta_heading;
    double resid_x = pred_speed*cos(pred_heading)-current_state.speed*cos(current_state.heading);
    double resid_y = pred_speed*sin(pred_heading)-current_state.speed*sin(current_state.heading);
//    double resid_x = current_state.speed*cos(current_state.heading)-current_state.speed*cos(current_state.heading);
//    double resid_y = current_state.speed*cos(current_state.heading)-current_state.speed*sin(current_state.heading);
//    ROS_ERROR("(%f, %f, %f, %f)", delta_speed, delta_heading, resid_x, resid_y);
    double log_likelihood = swarm_tools::log_multivariate_normal_2d(resid_x, resid_y, 0.0, 0.0, non_threat_sigma);
    return log_likelihood;
}
double calculate_threat_log_likelihood(agent::ObservedIntruderAgent intruder, const std::vector<agent::AgentState> &obstacles){
//    ROS_INFO("Calculating threat ll");
    agent::AgentState current_state = intruder.previous_states.front();
    intruder.previous_states.pop_front();
    agent::AgentState previous_state = intruder.previous_states.front();
    intruder.previous_states.pop_front();
    intruder.set_state(previous_state);
    agent::AgentCommand command;
//    intruder.set_aggression(1.0);
    int num_samples=1;
    double log_likelihoods[num_samples];
    double summed_log_likelihoods=0;
    double log_likelihood;
    for(int i=0;i<num_samples; i++){
        std::normal_distribution<double> distribution(0,0);

//        double mg_x = distribution(generator);
//        double mg_y = distribution(generator);

        swarm_control::get_intruder_command_from_motion_goal(intruder, agent::MotionGoal(0, 0), command);
        collision_avoidance::correct_command(intruder, obstacles, command);

        double pred_speed = previous_state.speed + command.delta_speed;
        double pred_heading = previous_state.heading + command.delta_heading;

        double resid_x = pred_speed*cos(pred_heading)-current_state.speed*cos(current_state.heading);
        double resid_y = pred_speed*sin(pred_heading)-current_state.speed*sin(current_state.heading);

        log_likelihood= swarm_tools::log_multivariate_normal_2d(resid_x, resid_y, 0, 0, threat_sigma);
        ROS_DEBUG("Threat (%f, %f, %f, %f, %f)", command.delta_speed, command.delta_heading, resid_x, resid_y, log_likelihood);
    }
//    double log_likelihood = summed_log_likelihoods/num_samples;
    return log_likelihood;
}

//double update_intruder_threat_probability(agent::ObservedIntruderAgent &intruder){
//    double threat_ll = calculate_threat_log_likelihood(intruder);
//    double non_threat_ll = calculate_non_threat_log_likelihood(intruder);
////    intruder.update_threat_estimate(threat_ll, non_threat_ll);
//}

void publish_threat_statistics(int intruder_id, double probability, bool classification){
    swarm_msgs::threatStatistics stats;
    stats.intruder_id=intruder_id;
    stats.threat_probability = probability;
    stats.threat_classification =classification;
    threat_pub.publish(stats);
}

void perception_callback(const swarm_msgs::worldState::ConstPtr& world_state_ptr){
    if(!initialized){
        return;
    }
    clock_t perc_t = clock();
    mtx.lock();
    agent::USVSwarm swarm_cp = swarm;

    ROS_INFO("USV %d Perception Callback", usv_id);
    usv_state_map.clear();
    intruder_state_map.clear();

    clock_t t = clock();
    extract_from_world_msg(world_state_ptr,
                           usv_state_map,
                           intruder_state_map,
                           asset_state);
    mtx.unlock();
    ROS_DEBUG("World Message Extraction time %f", (clock()-t)/(double) CLOCKS_PER_SEC);

    if(count_since_shuffle>10){
        swarm_cp.shuffle_tasks();
        if (usv_id==1){
            agent::SwarmAssignment assignment;
            swarm_cp.get_swarm_assignment(assignment);
            task_allocation_pub.publish(convert_to_swarm_assignment_msg(assignment));
        }
        count_since_shuffle=0;
    }
    std::vector<agent::AgentState> obstacle_states;
    swarm_cp.get_obstacle_states(obstacle_states);

    for(const auto &usv_state_id_pair : usv_state_map){
        if(swarm_cp.contains_usv(usv_state_id_pair.first)){
            t = clock();
            swarm_cp.update_usv_state_estimate(usv_state_id_pair.second);
            ROS_DEBUG("Updating usv %d time %f", usv_state_id_pair.first, (clock()-t)/(double) CLOCKS_PER_SEC);
        }
        else{
            ROS_INFO("Adding new usv %d to swarm", usv_state_id_pair.first);
            swarm_cp.add_usv(agent::USVAgent(usv_state_id_pair.second,
                   usv_constraints_map[usv_state_id_pair.first],
                   usv_radar_params_map[usv_state_id_pair.first],
                   agent::AgentAssignment()));
        }
    }

    for(const auto &intruder_state_id_pair : intruder_state_map){
        if(swarm_cp.contains_intruder(intruder_state_id_pair.first)){
            t = clock();
            swarm_cp.update_intruder_state_estimate(intruder_state_id_pair.second);
            ROS_DEBUG("Updating intruder %d : time %f", intruder_state_id_pair.first, (clock()-t)/(double) CLOCKS_PER_SEC);
        }
        else{
            ROS_INFO("Adding new intruder %d", intruder_state_id_pair.first);
           swarm_cp.add_intruder(agent::ObservedIntruderAgent(intruder_state_id_pair.second,
                   intruder_constraints_map[intruder_state_id_pair.first],
                   intruder_radar_params_map[intruder_state_id_pair.first]));
        }
        if(vanilla_threat_detection){
            swarm_cp.update_intruder_threat_estimate(intruder_state_id_pair.first);
        }else{
            auto intruder = swarm_cp.get_intruder_estimate_by_id(intruder_state_id_pair.first);
            if(intruder.previous_states.size()<2) continue;
            t = clock();
            double threat_ll = calculate_threat_log_likelihood(intruder, obstacle_states);
            double non_threat_ll = calculate_non_threat_log_likelihood(intruder, obstacle_states);
//            ROS_ERROR("LL (%f, %f)", threat_ll, non_threat_ll);
            swarm_cp.update_intruder_threat_estimate(intruder_state_id_pair.first, threat_ll, non_threat_ll);
            ROS_DEBUG("Updating intruder threat prob %d : time %f", intruder_state_id_pair.first, (clock()-t)/(double) CLOCKS_PER_SEC);
        }
    }
    t = clock();
    swarm_cp.update_queue_priorities();
    ROS_DEBUG("Update queue priorities time %f", (clock()-t)/(double) CLOCKS_PER_SEC);
    t = clock();
    for (const auto &task : swarm_cp.get_assignment_by_id(usv_id)){
        if (task.task_type != agent::TaskType::Guard && task.task_idx!=-1){
            const auto intruder = swarm_cp.get_intruder_estimate_by_id(task.task_idx);
            double probability = intruder.get_threat_probability();
            bool classification = intruder.is_threat();
            publish_threat_statistics(task.task_idx, probability, classification);
        }
    }
    ROS_DEBUG("Publishing threat_classification statistics total time %f", (clock()-t)/(double) CLOCKS_PER_SEC);


    // swarm_cp.update_intruder_estimates(intruder_state_map);

    agent::USVAgent usv = swarm_cp.get_usv_estimate_by_id(usv_id);

    agent::MotionGoal motion_goal;

    double aggression = swarm_control::get_motion_goal_from_assignment(usv_id,
                                                   swarm_cp,
                                                   motion_goal);
    usv.set_aggression(aggression);
    ROS_INFO("Before");
    publish_markers(motion_goal);

    agent::AgentCommand command;
    ROS_INFO("After");
    t = clock();
    ROS_INFO("Before1");
    swarm_cp.get_obstacle_states(obstacle_states);
    ROS_INFO("After1");
    ROS_INFO("Before2");
    swarm_control::get_usv_command_from_motion_goal(usv_id,
                                                    swarm_cp,
                                                    motion_goal,
                                                    command);
    ROS_INFO("After2");
    ROS_DEBUG("Motion goal calculation time %f", (clock()-t)/(double) CLOCKS_PER_SEC);
    ROS_DEBUG("DERSIRED COMMAND: (%f, %f)", command.delta_speed, command.delta_heading*180/swarm_tools::PI);

    t = clock();
    collision_avoidance::correct_command(usv,
                                         obstacle_states,
                                         command);
    ROS_INFO("Collision avoidance total time %f", (clock()-t)/(double) CLOCKS_PER_SEC);
    ROS_INFO("CORRECTED COMMAND: (%f, %f)", command.delta_speed, command.delta_heading*180/swarm_tools::PI);

    swarm_msgs::agentCommand command_msg = swarm_msgs::agentCommand();
    command_msg.delta_heading = command.delta_heading;
    command_msg.delta_speed = command.delta_speed;
    command_msg.sim_id = usv.get_sim_id();
    command_pub.publish(command_msg);
    ROS_INFO("Perception Callback time %f", (clock()-perc_t)/(double) CLOCKS_PER_SEC);
    try{
        std::lock_guard<std::mutex> lock(mtx);
        if(initialized){
            swarm = swarm_cp;
        }
    }catch(std::exception &e){
        ROS_ERROR("Copy Error %s", e.what());
    }
    count_since_shuffle++;
} // callback


void task_allocator_callback(const swarm_msgs::swarmAssignment::ConstPtr &swarm_task_assignment_ptr){
    std::lock_guard<std::mutex> lock(mtx);
    if(!initialized) {
        ROS_INFO("Can't allocate tasks, swarm not initialized");
        return;
    }
    if(swarm.block_next_task_allocation){
        ROS_INFO("Current task allocation is stale, waiting on re-evaluation");
        return;
    }
    ROS_INFO("Setting new task allocation!");
    agent::SwarmAssignment swarm_assignment = extract_from_swarm_assignment_msg(*swarm_task_assignment_ptr);
    ROS_INFO("%s", agent::swarm_assignment_to_string(swarm_assignment).c_str());
    try{
        swarm.update_swarm_assignment(swarm_assignment);
    }catch(std::exception &e){
        throw "Task Allocation Failed.";
    }
}

bool model_predictive_response(swarm_task_manager::modelPredictiveSimulation::Request &req,
                               swarm_task_manager::modelPredictiveSimulation::Response &res){
    if(swarm.get_num_usvs()==0 || !initialized){
        ROS_INFO("Can't do model prediction, swarm not initialized");
        return false;
    }

    mtx.lock();
    swarm.block_next_task_allocation=false;
    agent::USVSwarm swarm_cp = swarm;
    mtx.unlock();
    int num_timesteps_lookahead=req.num_timesteps_lookahead;
    double delta_time_secs = req.delta_time_secs;
    double threshold = req.threshold;
    agent::WeightedSwarmAssignment weighted_swarm_assignment;
    ROS_INFO("Model predictive request (%d, %f, %f)",
                num_timesteps_lookahead,
                delta_time_secs,
                threshold);
    try{
        agent::WeightedSwarmAssignment weighted_swarm_assignment;
         swarm_task_manager::get_best_candidate_swarm_assignment(usv_id,
                                                                 swarm_cp,
                                                                 usv_communication_map,
                                                                 num_timesteps_lookahead,
                                                                 delta_time_secs,
                                                                 threshold,
                                                                 weighted_swarm_assignment);

         ROS_INFO("Found best swarm assignment");
        res.candidate_assignment = convert_to_swarm_assignment_msg(weighted_swarm_assignment.first);
        res.weight = weighted_swarm_assignment.second;
    }catch(std::exception& e){
        ROS_ERROR("Task Allocation ERROR: %s", e.what());
        return false;
    }
    auto swarm_assignment = weighted_swarm_assignment.first;
    ROS_INFO("Submitting Swarm Assignment");
    ROS_INFO("%s", agent::swarm_assignment_to_string(swarm_assignment).c_str());
    ROS_INFO("WEIGHT: %f", weighted_swarm_assignment.second);
    ROS_INFO("Model Predictive Successful");
    return true;
}

void reset_callback(const swarm_msgs::resetSystem &msg){
    std::lock_guard<std::mutex> lock(mtx);
    if(!initialized) return;
    initialized=false;
    ROS_INFO("Resetting USV %d", usv_id);
    usv_radar_params_map.clear();
    usv_constraints_map.clear();
    intruder_radar_params_map.clear();
    intruder_constraints_map.clear();
    swarm.reset();
}

void get_agent_parameters(){

    // Get USV Parameters
    ROS_INFO("LOADING USV PARAMETERS");
    std::string usv_head_str = "/swarm_simulation/usv_params";
    get_agent_parameters(ros_container_ptr, usv_head_str, usv_constraints_map, usv_radar_params_map);

    // Get Intruder Parameters
    ROS_INFO("LOADING INTRUDER PARAMETERS");
    std::string intruder_head_str = "/swarm_simulation/intruder_params";
    get_agent_parameters(ros_container_ptr, intruder_head_str, intruder_constraints_map, intruder_radar_params_map);

    if (!ros_container_ptr->nh.getParam("/swarm_simulation/threat_sigma", threat_sigma)){
        ROS_ERROR("Could not load threat sigma");
    }
    if (!ros_container_ptr->nh.getParam("/swarm_simulation/non_threat_sigma", non_threat_sigma)){
        ROS_ERROR("Could not load non threat sigma");
    }
}
void initialize_callback(const swarm_msgs::initializeSystem &msg){
    std::lock_guard<std::mutex> lock(mtx);
    if (initialized) return;
    for(auto id : msg.usv_ids){
        if(id==usv_id){
            initialized = true;
            ROS_INFO("Initializing USV %d", usv_id);
            get_agent_parameters();
        }
    }
}


int main(int argc, char **argv){
    ros_container_ptr.reset(new RosContainer(argc, argv, "usv_planner"));
    ros::NodeHandle nh_priv("~");
    ROS_INFO("USV NODE STARTED");
    if(!nh_priv.getParam("usv_id", usv_id)){
        ROS_ERROR("USV ID NOT FOUND");
        usv_id=1;
    }
    else{ROS_INFO("Found USV ID %d IN PARAMETERS", usv_id);}
    // Initialize swarm
    swarm.set_main_usv_id(usv_id);
    auto threat_detection_client = ros_container_ptr->nh.serviceClient<swarm_threat_detection::ThreatDetection>("ThreatDetection");
    auto intruder_model_client = ros_container_ptr->nh.serviceClient<swarm_threat_detection::batchIntruderCommands>("swarm_intruder_model_service");
    swarm.set_threat_detection_client(threat_detection_client);
    swarm.intruder_model_client=intruder_model_client;

    // Command and motion goal marker publisher
    command_pub = ros_container_ptr->nh.advertise<swarm_msgs::agentCommand>("Commands", 1000);
    marker_pub = ros_container_ptr->nh.advertise<swarm_msgs::simulationMarker>("Markers", 1000);
    threat_pub = ros_container_ptr->nh.advertise<swarm_msgs::threatStatistics>("threatStatistics", 1000);
    task_allocation_pub = ros_container_ptr->nh.advertise<swarm_msgs::swarmAssignment>("Task_Allocation", 1000);

    // Perception Subscriber
    ros::Subscriber perc_sub = ros_container_ptr->nh.subscribe("Perception", 1000, perception_callback);
    ros::Subscriber task_sub = ros_container_ptr->nh.subscribe("Task_Allocation", 1000, task_allocator_callback);
    ros::Subscriber reset_sub = ros_container_ptr->nh.subscribe("SystemReset", 100, reset_callback);
    ros::Subscriber initialize_sub = ros_container_ptr->nh.subscribe("SystemStart", 100, initialize_callback);

    // Advertise sync service
    auto sync_name = boost::format("sync_service_%d") % usv_id;
    auto mp_name = boost::format("mp_simulation_service_%d") % usv_id;
    ros::ServiceServer mp_service = ros_container_ptr->nh.advertiseService(mp_name.str(), model_predictive_response);

    // Spin until initialized
    ros::Rate loop_rate(100);
    while(ros::ok() && !initialized) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Initialized %d", initialized);
    if(!ros::ok()) return 0;
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    // ros::spin();

    return 0;
}