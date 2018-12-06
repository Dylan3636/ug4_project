#include "ros_swarm_tools.h"
#include "task_allocation.h"
#include "swarm_task_manager/modelPredictiveSimulation.h"

int NUM_USVS = 4;
ros::Publisher task_allocation_publisher;
std::map<int, ros::ServiceClient> service_client_map;
void reallocate_tasks(){
    int usv_id;
    swarm_task_manager::modelPredictiveSimulation srv;
    swarm_task_manager::modelPredictiveSimulation::Request res_msg;
    std::vector<agent::WeightedSwarmAssignment> weighted_assignments;

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
            continue;
        }
    }
    if (weighted_assignments.size()==0){
        return;
    }
    auto local_max_weighted_assignment = swarm_task_manager::max_weighted_swarm_assignment(weighted_assignments);
    ROS_INFO("Weight: %f", local_max_weighted_assignment.second);
    auto swarm_assignment_msg = convert_to_swarm_assignment_msg(local_max_weighted_assignment.first);
    task_allocation_publisher.publish(swarm_assignment_msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Task_Manager");
    ros::NodeHandle n;
    task_allocation_publisher = n.advertise<swarm_msgs::swarmAssignment>("Task_Allocation", 1000);
    int usv_id;
    for(int i=0; i<NUM_USVS; i++){
        usv_id = i;
        auto mp_name = boost::format("mp_simulation_service_%d") % usv_id;
        service_client_map[usv_id] = n.serviceClient<swarm_task_manager::modelPredictiveSimulation>(mp_name.str());
    }
    ros::Rate loop_rate(10);
    while(ros::ok){
        reallocate_tasks();
        loop_rate.sleep();
    }

}