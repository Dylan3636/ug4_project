#include "swarm_tools.h"
#include "ros/ros.h"
#include <vector>
#include <deque>
#include <map>
#include <random>
#include <time.h>
#ifndef AGENT_H
#define AGENT_H

namespace agent{
    enum AgentType{
        USV,
        Intruder,
        Static,
        Asset,
        Base
    };
    struct AgentCommand
    {
        double delta_speed=0.0;
        double delta_heading=0.0;
        AgentCommand()=default;
        AgentCommand(const AgentCommand &command)=default;
        AgentCommand(double delta_speed, double delta_heading) : delta_speed(delta_speed), delta_heading(delta_heading){}
    };
    struct AgentState
    {
        double x;
        double y;
        double speed;
        double heading;
        double radius;
        int sim_id;

        swarm_tools::Point2D get_position() const{
            return swarm_tools::Point2D{x, y};
        }

        AgentState(const AgentState &state){
//            ROS_DEBUG("Copy State Constructor");
            x = state.x;
            y = state.y;
            speed = state.speed;
            heading = state.heading;
            radius = state.radius;
            sim_id = state.sim_id;
        }

        AgentState(){
//            ROS_DEBUG("Default State Constructor");
            x=0;
            y=0;
            speed=0;
            heading=0;
            radius=-1;
            sim_id=-500;
        }

        AgentState(double x,
                   double y,
                   double speed,
                   double heading,
                   double radius,
                   int sim_id) : x(x), y(y), speed(speed), heading(heading), radius(radius), sim_id(sim_id){
        }
    };

    struct AgentConstraints
    {
        double max_speed;
        double max_delta_speed;
        double max_delta_heading;
    };

    enum TaskType{
        Delay,
        Guard,
        Observe
    };

    struct AgentTask{
        TaskType task_type=Guard;
        int task_idx=-1; // ID of intruder or guard position

        AgentTask(){
            task_idx=-1;
            task_type=Guard;
        }
        AgentTask(const AgentTask &task){
            task_idx=task.task_idx;
            task_type=task.task_type;
        }
        AgentTask(TaskType tt, int tidx){
            task_type=tt;
            task_idx=tidx;
        }
        bool operator==(const AgentTask &task) const{
            return task_type==task.task_type && task_idx==task.task_idx;
        }
        std::string to_string()const{
            std::string task_str="Task: ";
            switch (task_type){
                case Delay:
                    task_str+="Delay";
                    break;
                case Guard:
                    task_str+="Guard";
                    break;
                case Observe:
                    task_str+="Observe";
                    break;
                default:
                    task_str+="TASK NOT SET";
                    break;
            }
            task_str += " Index: ";
            task_str += std::to_string(task_idx);
            task_str += "\n";
            return task_str;
        }
    };
    struct WeightedTask : AgentTask{
        double weight=0;

        WeightedTask():AgentTask() {
//            ROS_DEBUG("Default Weighted Constructor");
        }
        WeightedTask(TaskType tt, int tidx, double weight) : AgentTask(tt, tidx), weight(weight){}
        WeightedTask(AgentTask task, double weight) : AgentTask(task), weight(weight){}
        WeightedTask(const WeightedTask &wt) : AgentTask(wt), weight(wt.weight){
//           ROS_DEBUG("Copy Weighted Constructor");
        }

        bool operator<(const WeightedTask &wt) const{
            return weight < wt.weight;
        }
        bool operator==(const WeightedTask &wt) const{
            return weight < wt.weight;
        }
        std::string to_string() const{
            auto task_str = AgentTask::to_string();
            task_str += "Weight: ";
            task_str += std::to_string(weight);
            task_str += "\n";
            return task_str;
        }
    };
    typedef std::vector<AgentTask> AgentAssignment;

    std::string agent_assignment_to_string(const AgentAssignment &assignment){
        std::string str;
        for(const auto &task : assignment){
            str += task.to_string();
        }
        return str;

    }

    struct CollisionAvoidanceParameters
    {
        double max_radar_distance{100};
        double max_radar_angle_rad{swarm_tools::PI/2};
        double aggression{0.5};
        CollisionAvoidanceParameters()= default;
        CollisionAvoidanceParameters(double max_radar_distance, double max_radar_angle_rad, double aggression)
                :max_radar_distance(max_radar_distance), max_radar_angle_rad(max_radar_angle_rad),
                 aggression(aggression) {}
    };

    struct MotionGoal{
        double x{0};
        double y{0};
        double speed{0};
        double heading_rad{0};
        swarm_tools::Point2D get_position() const{
            return swarm_tools::Point2D{x, y};
        }

        MotionGoal() = default;

        MotionGoal(double x, double y) : x(x), y(y){
            speed = 0;
            heading_rad = 0;
        }

        explicit MotionGoal(const swarm_tools::Point2D &position){
            x = position.x;
            y = position.y;
            speed = 0;
            heading_rad = 0;
        }

        MotionGoal(double x,
                   double y,
                   double speed,
                   double heading_rad) : x(x), y(y),
                                         speed(speed),
                                         heading_rad(heading_rad) {}

    };

    typedef std::map<int, AgentAssignment> SwarmAssignment;
    typedef std::pair<SwarmAssignment, double> WeightedSwarmAssignment;

    std::string swarm_assignment_to_string(const SwarmAssignment swarm_assignment){
        std::string str;
        for(const auto &assignment_pair: swarm_assignment){
            str += "USV ";
            str += std::to_string(assignment_pair.first);
            str += ":\n";
            str += agent_assignment_to_string(assignment_pair.second);
        }
        return str;
    }

    class BaseAgent
    {
            AgentState state{};
            AgentConstraints constraints{};
            CollisionAvoidanceParameters ca_params{};
            AgentType type{Base};
            time_t previously_blocked_time = time(nullptr);
            bool _evading = false;
            std::default_random_engine generator;
        public:

            void update_previously_blocked_time(){
                time(&previously_blocked_time);
            }

            time_t get_previously_blocked_time(){
                return previously_blocked_time;
            }
            bool evade() const{
                return _evading;
            }
            bool update_evade(){
                if(_evading){
                    ROS_INFO("Currently evading");
                    double delta_time_secs = std::difftime(time(nullptr), previously_blocked_time);
                    if (delta_time_secs>1){
                        _evading=false;
                        return _evading;
                    }else{
                        return _evading;
                    }
                }
                double lambda = 5;
                double delta_time_secs = std::difftime(time(nullptr), previously_blocked_time);
                double cdf = 1-std::exp(-lambda*delta_time_secs);
                if(delta_time_secs<2){
                    std::bernoulli_distribution dist(get_aggression());
                    _evading = dist(generator);
                }
                ROS_INFO("Evade: (%d, %f. %f)", _evading, cdf, delta_time_secs);
                return _evading;
            }

            BaseAgent() = default;

            BaseAgent (const AgentState &state, AgentType type) : state(state), type(type){
                constraints={};
                ca_params={};
            }
            BaseAgent (const AgentState &state,
                       const AgentConstraints &constraints,
                       const CollisionAvoidanceParameters &ca_params,
                       AgentType type) : state(state),
                                         constraints(constraints),
                                         ca_params(ca_params),
                                         type(type){}

        BaseAgent(const BaseAgent &agent){
                set_state(agent.get_state());
                set_constraints(agent.get_constraints());
                set_collision_avoidance_params(agent.get_collision_avoidance_params());
                previously_blocked_time=agent.previously_blocked_time;
                _evading = agent._evading;
                generator = generator;
            }

            double get_radius() const{
                return state.radius;
            }

            double get_x() const{
                return state.x;
            }

            double get_y() const{
                return state.y;
            }

            double get_speed() const{
                return state.speed;
            }

            double get_heading() const{
                return state.heading;
            }

            int get_sim_id() const{
                return this->state.sim_id;
            }

            swarm_tools::Point2D get_position() const{
                return state.get_position();
            }

            AgentType get_agent_type() const{
                return type;
            }

            void set_agent_type(AgentType agent_type){
                type=agent_type;
            }

            AgentState get_state() const{
                return state;
            }

            virtual void set_state(const AgentState &new_state){
                state = new_state;
            }
            void update_state(double x, double y, double speed, double heading){
                set_state(AgentState(x, y, speed, heading, get_radius(), get_sim_id()));
            }
            double get_max_speed() const{
                return constraints.max_speed;
            }
            double get_max_delta_speed() const{
                return constraints.max_delta_speed;
            }
            double get_max_delta_heading() const{
                return constraints.max_delta_heading;
            }
            double get_max_radar_distance() const{
                return ca_params.max_radar_distance;
            }
            double get_max_radar_angle_rad() const{
                return ca_params.max_radar_angle_rad;
            }
            double get_aggression() const{
                return ca_params.aggression;
            }
            double set_aggression(double aggression){
                ca_params.aggression = aggression;
            }
            AgentConstraints get_constraints() const{
                return constraints;
            }

            void set_constraints(AgentConstraints new_constraints){
                constraints=new_constraints;
            }

            CollisionAvoidanceParameters get_collision_avoidance_params() const{
                return ca_params;
            }

            void set_collision_avoidance_params(
                CollisionAvoidanceParameters new_ca_params){
                ca_params=new_ca_params;
                }

            virtual void command_agent_forward(const AgentCommand &command,
                                       double delta_time_secs){

                state.speed += std::min(command.delta_speed*delta_time_secs, this->constraints.max_speed);
                state.speed = std::max(state.speed, 0.0);
                state.heading += command.delta_heading*delta_time_secs;
                state.heading = fmod(state.heading, 2*swarm_tools::PI);

                state.x += state.speed*cos(state.heading)*delta_time_secs;
                state.y += state.speed*sin(state.heading)*delta_time_secs;
        }
    };

    class IntruderAgent : public BaseAgent{
        bool threat_classification=false;
        public:
            int sequence_length = 10;
            int count = 0;
            std::deque<AgentState> previous_states;
            double heading_goal=0;

            IntruderAgent() = default;
            IntruderAgent(bool threat, const AgentState &initial_state) : BaseAgent(initial_state, Intruder){
                set_threat_classification(threat);
                set_state(initial_state);
                heading_goal=initial_state.heading;
            }
            IntruderAgent(const IntruderAgent &agent) : BaseAgent(agent){
                threat_classification=agent.threat_classification;
                sequence_length = agent.sequence_length;
                previous_states = agent.previous_states;
                count=agent.count;
                heading_goal = agent.heading_goal;
            }

            IntruderAgent(
                    bool threat,
                    const AgentState &initial_state,
                    const AgentConstraints &constraints,
                    const CollisionAvoidanceParameters &ca_params) : BaseAgent(initial_state,
                                                                                     constraints,
                                                                                     ca_params,
                                                                                     Intruder){
                set_state(initial_state);
                set_threat_classification(threat);
                heading_goal=initial_state.heading;
            }
            bool is_threat() const{
                return threat_classification;
            }
            void set_threat_classification(bool threat){
                threat_classification=threat;
            }
            void set_state(const agent::AgentState &state){
                BaseAgent::set_state(state);

                while (previous_states.size() >= sequence_length){
                    previous_states.pop_back();
                }
                previous_states.push_front(state);
            }

            void command_agent_forward(const AgentCommand &command,
                                       double delta_time_secs){
                BaseAgent::command_agent_forward(command, delta_time_secs);
                set_state(get_state());
            }

            };

//    class IntruderAgent : public BaseIntruderAgent{
//        public:

//            IntruderAgent() = default;
//            IntruderAgent(bool threat, const AgentState &initial_state) : threat_classification(threat),
//                                                                             BaseAgent(initial_state, Intruder){
//           }
//            IntruderAgent(const IntruderAgent &agent) : BaseAgent(agent){
//                threat_classification=agent.is_threat();
//            }
//
//            IntruderAgent(bool is_threat,
//                          const AgentState &initial_state,
//                          const AgentConstraints &constraints,
//                          const CollisionAvoidanceParameters &ca_params) : threat_classification(is_threat),
//                                                                           BaseAgent(initial_state,
//                                                                                     constraints,
//                                                                                     ca_params,
//                                                                                     Intruder){
//            }
//
//            void set_state(const agent::AgentState &state){
//                BaseAgent::set_state(state);
//                while (previous_states.size() >= sequence_length){
//                    previous_states.pop_back();
//                }
//                previous_states.push_front(state);
//            }

//    };

    class ObservedIntruderAgent : public IntruderAgent
    {   
        double threat_probability=0.05;
        double threat_threshold_upper=0.75;
        double threat_threshold_lower=0.25;

        public:

            void set_threat_estimate(bool threat, double probability){
                set_threat_classification(threat);
                threat_probability=probability;
            }
            int update_threat_estimate(double threat_likelihood, double non_threat_likelihood, double dist_to_asset){
                threat_probability += 1e-10;
                double trans_prob = exp(-0.01*dist_to_asset);
                double threat_joint = threat_likelihood*(trans_prob*(1-threat_probability) + threat_probability);
                double non_threat_joint =  non_threat_likelihood*(1-trans_prob)*(1-threat_probability);
                threat_probability = threat_joint/(threat_joint + non_threat_joint + 1e-10);
                ROS_DEBUG("(%d, %f, %f, %f, %f, %f)", get_sim_id(), threat_likelihood, non_threat_likelihood, threat_probability, trans_prob, dist_to_asset);
                if(std::isnan(threat_probability)){
                    threat_probability = 1e-10;
                }
                if(threat_probability>threat_threshold_upper){
                    set_threat_classification(true);
                    return 1;
                }
                if(threat_probability<threat_threshold_lower){
                    set_threat_classification(false);
                    return -1;
                }
                return 0;
            }
            bool get_threat_classification() const{
                return is_threat();
            }
            double get_threat_probability() const{
                return threat_probability;
            }

            bool sample_inplace(std::default_random_engine generator){
                if (!is_threat()){
                    std::bernoulli_distribution dist(this->threat_probability);
                    set_threat_classification(dist(generator));
                }
                return is_threat();
            }
            bool sample(std::default_random_engine generator) const{
                bool threat;
                if (!is_threat()){
                    std::bernoulli_distribution dist(this->threat_probability);
                    threat = dist(generator);
                }else{
                    threat=true;
                }
                return threat;
            }

            ObservedIntruderAgent()=default;
            ObservedIntruderAgent(const AgentState &state,
                          const AgentConstraints &constraints,
                          const CollisionAvoidanceParameters &ca_params)
                    : IntruderAgent(false, state, constraints, ca_params){
                threat_probability=0.05;
            }
            ObservedIntruderAgent(const ObservedIntruderAgent &agent) : IntruderAgent(agent){
                threat_probability=agent.get_threat_probability();
                double threat_threshold_upper=agent.threat_threshold_upper;
                double threat_threshold_lower=agent.threat_threshold_lower;
            }
            explicit ObservedIntruderAgent(const AgentState &state){
                set_state(state);
                set_constraints(AgentConstraints{35, 5, swarm_tools::PI/2});
                set_collision_avoidance_params(CollisionAvoidanceParameters{50, swarm_tools::PI, 0.5});
                set_agent_type(Intruder);
                set_threat_classification(false);
                threat_probability=0.05;
            }
    };

    struct AssetAgent : public BaseAgent
    {
        public:
            AssetAgent()=default;
            AssetAgent(const AgentState &state){
                set_state(state);
                set_agent_type(Asset);
            }
            AssetAgent(const AssetAgent &asset)=default;
    };

    double get_observation_weight(const agent::ObservedIntruderAgent &intruder, const agent::AssetAgent &asset){
        double w_obs = 300;
        double w_dist = 1000;
        double dist_to_asset = swarm_tools::euclidean_distance(asset.get_position(),
                                                               intruder.get_position());
        double p_threat = intruder.get_threat_probability();
        // w_obs * p_threat * (1 + w_dist / dist_to_asset);
        return  w_obs*p_threat;
    }
    double get_guard_weight() {
        return 100;
    }

    class USVAgent : public BaseAgent
    {
        // private variables
        AgentAssignment current_assignment;
        bool _has_delay_task=false;
        int max_num_tasks=4;
        int current_num_tasks=0;
        public:


//            double get_aggression() const{
//                if (has_delay_task()){
//                    return 1;
//                }else {
//                    return BaseAgent::get_aggression();
//                }
//
//            }

        AgentAssignment get_current_assignment() const{
                return current_assignment;
            }

            void clear_current_assignment(){
                current_assignment.clear();
            }

            void set_current_assignment(const AgentAssignment &assignment){
                current_assignment.clear();
                current_num_tasks=0;
                _has_delay_task=false;

                for(const auto &task : assignment){
                    if(task.task_idx==-1) continue;
                    if(task.task_type==Delay){
                        _has_delay_task=true;
                        current_assignment.push_back(task);
                    }else if(task.task_type==Observe){
                        add_observe_task(task.task_idx);
                    }else{
                        add_guard_task(task.task_idx);
                    }
                }
            }

            bool switch_observe_to_delay_task(int intruder_id){
                if(has_delay_task()){
                    return false;
                }else{
                    set_delay_assignment(intruder_id);
                    return true;
                }
                // bool had_observe_task = remove_observe_task(intruder_id);
            }
            bool switch_delay_to_observe_task(int intruder_id){
                if(!has_delay_task()){
                    return false;
                }else{
                    for(auto &task: current_assignment){
                        if(task.task_type==TaskType::Delay && task.task_idx==intruder_id){
                            task.task_type=TaskType::Observe;
                        }
                    }
                }
                // bool had_observe_task = remove_observe_task(intruder_id);
            }

            void set_delay_assignment(int delay_assignment_idx){
                bool set=false;
                if(delay_assignment_idx==-1) return;
                for(auto &task : current_assignment){
                    if(task.task_type==Delay){
                        task.task_idx=delay_assignment_idx;
                        set=true;
                        break;
                    } else if(task.task_type==Observe && task.task_idx==delay_assignment_idx){
                        task.task_idx=delay_assignment_idx;
                        task.task_type=Delay;
                        set=true;
                        break;
                        
                    }
                }
                if(!set){
                   current_assignment.push_back(AgentTask(Delay, delay_assignment_idx));
                   _has_delay_task=true;
                   current_num_tasks++;
                }

            }

            bool add_guard_task(int guard_assignment_idx){
                if(guard_assignment_idx==-1) return false;
                remove_guard_task(guard_assignment_idx);
                if(current_num_tasks>=max_num_tasks) return false;
                current_assignment.emplace_back(Guard, guard_assignment_idx);
                current_num_tasks++;
                return true;
            }
            
            bool add_observe_task(int observe_assignment_idx){
                ROS_DEBUG("Attemptind to add observe task %d", observe_assignment_idx);
                ROS_DEBUG("Current num observations %d", current_num_tasks);
                ROS_DEBUG("Max num observations %d", max_num_tasks);
                if(observe_assignment_idx==-1) return false;
                remove_observe_task(observe_assignment_idx);
                if(current_num_tasks>=max_num_tasks) return false;
                remove_observe_task(observe_assignment_idx);
                current_assignment.emplace_back(Observe, observe_assignment_idx);
                current_num_tasks++;
                return true;
            }
            bool has_delay_task() const{
                return _has_delay_task;
//                for(const auto &task : current_assignment){
//                    if(task.task_type==Delay && task.task_idx!=-1) return true;
//                }
//                return false;
            }
            bool has_delay_task(int intruder_id) const{
                for (const auto &task : current_assignment) {
                    if (task.task_type == Delay && task.task_idx == intruder_id) return true;
                }
                return false;
            }
            void pop_all_non_priority_tasks(std::vector<AgentTask> &popped_tasks){
                popped_tasks.clear();
                for(int i =0; i < current_assignment.size(); i++){
                    if(current_assignment[i].task_type!=Delay){
                        popped_tasks.emplace_back(
                                current_assignment[i].task_type,current_assignment[i].task_idx);
                        current_assignment[i]=current_assignment.back();
                        current_assignment.pop_back();
                        current_num_tasks--;
                    }
                }
            }
            bool remove_observe_task(int observe_assignment_idx){
                ROS_DEBUG("Attempting to remove observe task %d", observe_assignment_idx);
                auto task2rem = AgentTask(Observe, observe_assignment_idx);
                for(int i =0; i < current_assignment.size(); i++){
                    if(current_assignment[i].task_type==Observe && current_assignment[i].task_idx==observe_assignment_idx){
                        current_assignment[i]=current_assignment.back();
                        current_assignment.pop_back();
                        current_num_tasks--;
                        return true;
                    }
                }
                return false;
            }
            bool remove_guard_task(int guard_assignment_idx){
                for(int i =0; i < current_assignment.size(); i++){
                    if(current_assignment[i].task_type==Guard && current_assignment[i].task_idx==guard_assignment_idx){
                        current_assignment[i]=current_assignment.back();
                        current_assignment.pop_back();
                        current_num_tasks--;
                        return true;
                    }
                }
                return false;
            }
            bool swap_tasks(agent::AgentTask old_task, agent::AgentTask new_task){
                if(new_task.task_idx==-1) return false;
                if(TaskType::Observe == old_task.task_type){
                    bool result = remove_observe_task(old_task.task_idx);
                    if(!result) return false;
                }else if(agent::TaskType::Guard == old_task.task_type){
                    remove_guard_task(old_task.task_idx);
                }else{
                    return false;
                }
                if(TaskType::Observe == new_task.task_type){
                    add_observe_task(new_task.task_idx);
                }else if(agent::TaskType::Guard == old_task.task_type){
                    add_guard_task(old_task.task_idx);
                }else{
                    return false;
                }
                return true;
            }
            bool swap_observe_tasks(int old_task_idx, int new_task_idx){
                bool result = remove_observe_task(old_task_idx);
                if(!result) return false;
                add_observe_task(new_task_idx);
                return true;
            }
            USVAgent() = default;
            USVAgent(const AgentState &state,
                     const AgentConstraints &constraints,
                     const CollisionAvoidanceParameters &ca_params,
                     const AgentAssignment &assignment)
                    : BaseAgent(state, constraints, ca_params, USV)
                {
                    set_current_assignment(assignment);
                }

            explicit USVAgent(const AgentState &state){
                set_state(state);
                set_constraints(AgentConstraints{40,
                                                100,
                                                swarm_tools::PI/2});
                set_collision_avoidance_params(
                    CollisionAvoidanceParameters{25,
                                                swarm_tools::PI,
                                                0.9});
                set_current_assignment(AgentAssignment());
                set_agent_type(USV);
            }
            USVAgent(const USVAgent &usv):BaseAgent(usv){
                _has_delay_task = usv.has_delay_task();
                max_num_tasks=usv.max_num_tasks;
                current_num_tasks=usv.current_num_tasks;
                set_current_assignment(usv.get_current_assignment());
            }
    };


    int get_left_and_right_points();
}
#endif