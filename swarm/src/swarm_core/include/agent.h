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
        double delta_speed;
        double delta_heading;
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
            x = state.x;
            y = state.y;
            speed = state.speed;
            heading = state.heading;
            radius = state.radius;
            sim_id = state.sim_id;
        }

        AgentState(){
            x=0;
            y=0;
            speed=0;
            heading=0;
            radius=-1;
            sim_id=-1;
        }

        AgentState(double x,
                   double y,
                   double speed,
                   double heading,
                   double radius,
                   int sim_id){
            this->x=x;
            this->y=y;
            this->speed=speed;
            this->heading=heading;
            this->radius=radius;
            this->sim_id=sim_id;
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
        TaskType task_type;
        int task_idx=-1; // ID of intruder or guard position

        AgentTask(){
            task_idx=-1;
            task_type=Delay;
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
            time_t previously_blocked_time = time(NULL);

        public:

            void update_previously_blocked_time(){
                time(&previously_blocked_time);
            }

            time_t get_previously_blocked_time(){
                return previously_blocked_time;
            }
            bool evade() const{
                std::default_random_engine generator;
                double lambda = 5;
                double delta_time_secs = std::difftime(time(NULL), previously_blocked_time);
                double pdf = lambda*std::exp(-lambda*delta_time_secs);
                std::bernoulli_distribution dist(pdf);
                return dist(generator);
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

            void copy(const BaseAgent &agent){
                set_state(agent.get_state());
                set_constraints(agent.get_constraints());
                set_collision_avoidance_params(agent.get_collision_avoidance_params());
            }
            BaseAgent(const BaseAgent &agent){
                set_state(agent.get_state());
                set_constraints(agent.get_constraints());
                set_collision_avoidance_params(agent.get_collision_avoidance_params());
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
                state = AgentState(new_state);
            }
            void update_state(double x, double y, double speed, double heading){
                set_state(AgentState{x, y, speed, heading, get_radius(), get_sim_id()});
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

            void command_agent_forward(const AgentCommand &command,
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
            int count =0;
            std::deque<AgentState> previous_states;

            IntruderAgent() = default;
            IntruderAgent(bool threat, const AgentState &initial_state) : BaseAgent(initial_state, Intruder){
                set_threat_classification(threat);
            }
            IntruderAgent(const IntruderAgent &agent) : BaseAgent(agent){
                threat_classification=agent.threat_classification;
                sequence_length = agent.sequence_length;
                previous_states = agent.previous_states;
            }

            IntruderAgent(
                    bool threat,
                    const AgentState &initial_state,
                    const AgentConstraints &constraints,
                    const CollisionAvoidanceParameters &ca_params) : BaseAgent(initial_state,
                                                                                     constraints,
                                                                                     ca_params,
                                                                                     Intruder){
            set_threat_classification(threat);
            }
            bool is_threat() const{
                return threat_classification;
            }
            void set_threat_classification(bool threat){
                threat_classification=threat;
            }
            void set_state(const agent::AgentState &state){
                BaseAgent::set_state(state);
                count++;
                if (count%1!=0){
                    return;
                }else{
                    count=0;
                }
                while (previous_states.size() >= sequence_length){
                    previous_states.pop_back();
                }
                previous_states.push_front(state);
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

        public:

            void set_threat_estimate(bool threat, double probability){
                set_threat_classification(threat);
                threat_probability=probability;
            }
            bool get_threat_classification() const{
                return is_threat();
            }
            double get_threat_probability() const{
                return threat_probability;
            }

            bool sample(std::default_random_engine generator){
                if (!is_threat()){
                    std::bernoulli_distribution dist(this->threat_probability);
                    set_threat_classification(dist(generator));
                }
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
    };


    class USVAgent : public BaseAgent
    {
        // private variables
        AgentAssignment current_assignment;
        bool _has_delay_task=false;
        public:


//            double get_aggression() const{
//                if (has_delay_task()){
//                    return 1;
//                }else {
//                    return BaseAgent::get_aggression();
//                }
//
//            }

            void copy(const USVAgent &usv){
                BaseAgent::copy(usv);
                set_current_assignment(usv.get_current_assignment());
            }
            AgentAssignment get_current_assignment() const{
                return current_assignment;
            }

            void set_current_assignment(const AgentAssignment &assignment){
                current_assignment.clear();
                _has_delay_task=false;
                for(const auto &task : assignment){
                    if(task.task_type==Delay){
                        _has_delay_task=true;
                    }
                    current_assignment.push_back(task);
                }
            }
            bool switch_observe_to_delay_assignment(int intruder_id){
                if(has_delay_task()){
                    return false;
                }else{
                    set_delay_assignment(intruder_id);
                    return true;
                }
                // bool had_observe_task = remove_observe_assignment(intruder_id);

            }
            void set_delay_assignment(int delay_assignment_idx){
                bool set=false;
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
                }

            }
            void set_guard_assignment(int guard_assignment_idx){
                for(auto &task : current_assignment){
                    if(task.task_type==Guard){
                        task.task_idx=guard_assignment_idx;
                        return;
                    }
                }
                current_assignment.emplace_back(Guard, guard_assignment_idx);
            }
            
            void add_observe_task(int observe_assignment_idx){
                remove_observe_assignment(observe_assignment_idx);
                current_assignment.push_back(AgentTask(Observe, observe_assignment_idx));
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

            bool remove_observe_assignment(int observe_assignment_idx){
                auto task2rem = AgentTask(Observe, observe_assignment_idx);
                for(int i =0; i < current_assignment.size(); i++){
                    if(current_assignment[i].task_type==Observe && current_assignment[i].task_idx==observe_assignment_idx){
                        current_assignment[i]=current_assignment.back();
                        current_assignment.pop_back();
                        return true;
                    }
                }
                return false;
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
                set_current_assignment(usv.get_current_assignment());
                _has_delay_task = usv.has_delay_task();
            }
    };


    int get_left_and_right_points();
}
#endif