#include <ros/ros.h>
#include "ros_swarm_tools.h"
#include "task_allocation.h"
#include "swarm_task_manager/modelPredictiveSimulation.h"

int NUM_USVS;
RosContainerPtr ros_container_ptr;
ros::Publisher task_allocation_publisher;
std::map<int, ros::ServiceClient> service_client_map;
void reallocate_tasks(){
    ROS_INFO("Reallocating Tasks");
    int usv_id;
    swarm_task_manager::modelPredictiveSimulation srv;
    swarm_task_manager::modelPredictiveSimulation::Request res_msg;
    std::vector<agent::WeightedSwarmAssignment> weighted_assignments;

    bool failed = !ros_container_ptr->nh.getParam("/task_manager/delta_time_secs", srv.request.delta_time_secs);
    if(failed){
        ROS_ERROR("delta_time_parameter NOT FOUND");
    }
    failed = !ros_container_ptr->nh.getParam("/task_manager/num_timesteps_lookahead", srv.request.num_timesteps_lookahead);
    if(failed){
        ROS_ERROR("num_time_steps_lookahead NOT FOUND");
    }
    failed = !ros_container_ptr->nh.getParam("/task_manager/dist_to_asset_threshold", srv.request.threshold);
    if(failed){
        ROS_ERROR("dist_to_asset_threshold NOT FOUND");
    }

    agent::SwarmAssignment swarm_assignment_candidate;
    double weight;
    agent::WeightedSwarmAssignment weighted_candidate;
    for(int i=0; i<NUM_USVS; i++){
        usv_id = i;
        if(service_client_map[usv_id].call(srv)){
            swarm_assignment_candidate = extract_from_swarm_assignment_msg(srv.response.candidate_assignment);
            weight = srv.response.weight;
            weighted_candidate = std::pair<agent::SwarmAssignment, double>(swarm_assignment_candidate, weight);
            weighted_assignments.push_back(weighted_candidate);
        }
        else{
            ROS_INFO("Model Prediction Failed for usv %d", i);
            continue;
        }
    }
    if (weighted_assignments.empty()){
        ROS_INFO("No candidate assignments were returned");
        return;
    }
    for (const auto &weighted_assignment : weighted_assignments){
        ROS_INFO("Weight: %f", weighted_assignment.second);
        for (const auto &assignment_pair : weighted_assignment.first){
            ROS_INFO("%s", agent::agent_assignment_to_string(assignment_pair.second).c_str());
        }
    }
    auto local_max_weighted_assignment = swarm_task_manager::max_weighted_swarm_assignment(weighted_assignments);
    auto swarm_assignment_msg = convert_to_swarm_assignment_msg(local_max_weighted_assignment.first);
    task_allocation_publisher.publish(swarm_assignment_msg);
}

int main(int argc, char **argv){
    std::string s = "Task_Manager";
    ros_container_ptr.reset( new RosContainer(argc, argv, s.c_str()));
    task_allocation_publisher = ros_container_ptr->nh.advertise<swarm_msgs::swarmAssignment>("Task_Allocation", 1000);
    ros_container_ptr->nh.getParam("/swarm_simulation/num_usvs", NUM_USVS);
    int usv_id;
    for(int i=0; i<NUM_USVS; i++){
        usv_id = i;
        auto mp_name = boost::format("mp_simulation_service_%d") % usv_id;
        service_client_map[usv_id] = ros_container_ptr->nh.serviceClient<swarm_task_manager::modelPredictiveSimulation>(mp_name.str());
    }
    ros::Rate loop_rate(0.05);
    while(ros::ok){
        reallocate_tasks();
        loop_rate.sleep();
    }

}