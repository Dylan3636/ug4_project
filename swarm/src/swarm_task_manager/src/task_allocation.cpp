#include <vector>
#include <cmath>
#include "task_allocation.h"
#include "motion_goal_control.h"
#include "Hungarian.h"

// typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;
// typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

bool allocate_tasks(const std::vector<agent::USVAgent> &usvs,
                    const std::vector<agent::IntruderAgent> &intruders,
                    const agent::AssetAgent &asset,
                    std::vector<int> &assignments)
{
    int max_num_guard=4;
    double guard_radius = 80;
    const int num_usvs = static_cast<int>(usvs.size());
    const int num_intruders = static_cast<int>(intruders.size());
    int num_guards = std::min(max_num_guard, num_usvs);
    std::vector<std::vector<double>> costs;
    // VectorXd asset_distances;
    // intruder_distances(num_usvs, num_intruders);
    // asset_distances(num_usvs);
    double delay_weight;
    double asset_weight;
    double asset_delay_weight;
    double no_guard_weight;
    double intruder_guard_weight;

    for (int i=0; i<usvs.size(); i++){
        for (int j=0; j<intruders.size(); j++){
            agent::MotionGoal delay_motion_goal;
            usv_delay_motion_goal(usvs[i], intruders[j], asset, delay_motion_goal);

            double delay_cost = -1/swarm_tools::euclidean_distance(usvs[i].get_position(),
                                                                   delay_motion_goal.get_position());

            for(int k=0; k<=num_guards; k++){
                double asset_cost;
                double asset_delay_cost;
                if (k == num_guards){
                    asset_cost = 0.5 * no_guard_weight/asset_weight;
                    asset_delay_cost = 0.5 * no_guard_weight/asset_delay_weight;
                }else{
                    agent::MotionGoal guard_motion_goal;
                    usv_guard_motion_goal(num_usvs, k, guard_radius, asset.get_state(), guard_motion_goal);

                    asset_cost = -1/swarm_tools::euclidean_distance(usvs[i].get_position(),
                                                                    guard_motion_goal.get_position());

                    asset_delay_cost = -1/swarm_tools::euclidean_distance(guard_motion_goal.get_position(),
                                                                          delay_motion_goal.get_position());
 
                }
               // double intruder_guard_cost = swarm_tools::relative_angle_between_points(guard_motion_goal.position(),
                //                                                                             intruders[j].position(),
                //                                                                             2*M_PI*i/num_usvs) > 0;

                costs[i][j*num_guards+k] = delay_weight*delay_cost + asset_weight*asset_cost + asset_delay_weight*asset_delay_cost; // + intruder_guard_weight*intruder_guard_cost;
                }
               
            }
        }
    }
