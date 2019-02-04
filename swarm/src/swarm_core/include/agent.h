#include "swarm_tools.h"
#include "ros/ros.h"
#include <vector>
#include <map>
#ifndef AGENT_H
#define AGENT_H

namespace agent{
    enum AgentType{
        USV,
        Intruder,
        Static,
        Asset
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
        int task_idx; // ID of intruder or guard position

        void copy(const AgentTask &task){
            task_idx = task.task_idx;
            task_type = task.task_type;
        }
        AgentTask(){
            task_idx=-1;
            task_type=Delay;
        }
        AgentTask(const AgentTask &task){
            copy(task);
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
        double max_radar_distance;
        double max_radar_angle_rad;
        double aggression;
    };

    struct MotionGoal{
        double x;   
        double y;
        double speed;
        double heading_rad;
        swarm_tools::Point2D get_position() const{
            return swarm_tools::Point2D{this->x, this->y};
        }
    };

    typedef std::map<int, AgentAssignment> SwarmAssignment;
    typedef std::pair<SwarmAssignment, double> WeightedSwarmAssignment;

    std::string swarm_assignment_to_string(const SwarmAssignment swarm_assignment){
        std::string str = "";
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
            AgentState state;
            AgentConstraints constraints;
            CollisionAvoidanceParameters ca_params;

        public:
            AgentType type;

            BaseAgent(){}

            BaseAgent (const AgentState &state,
                       const AgentConstraints &constraints,
                       const CollisionAvoidanceParameters &ca_params,
                       AgentType type){
                set_state(state);
                set_constraints(constraints);
                set_collision_avoidance_params(ca_params);
                set_agent_type(type);
            }

            void copy(const BaseAgent &agent){
                set_state(agent.get_state());
                set_constraints(agent.get_constraints());
                set_collision_avoidance_params(agent.get_collision_avoidance_params());
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

            void set_state(const AgentState &new_state){
                state = AgentState(new_state);
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

    class IntruderAgent : public BaseAgent
    {   
        bool threat_classification;
        double threat_probability;

        public:

            void set_threat_estimate(bool alert, double probability){
                threat_classification=alert;
                threat_probability=probability;
            }

            double get_threat_probability(){
                return threat_probability;
            }
            bool is_threat(){
                return threat_classification;
            }

            IntruderAgent(){}
            IntruderAgent(AgentState state,
                          AgentConstraints constraints,
                          CollisionAvoidanceParameters ca_params)
                    : BaseAgent(state, constraints, ca_params, Intruder){
            }
            IntruderAgent(const AgentState &state){
                set_state(state);
                set_constraints(AgentConstraints{35, 5, swarm_tools::PI/2});
                set_collision_avoidance_params(CollisionAvoidanceParameters{50, swarm_tools::PI, 0.5});
                set_agent_type(Intruder);
            }
    };

    struct AssetAgent : public BaseAgent
    {
        public:
            AssetAgent(){};
            AssetAgent(const AgentState &state){
                set_state(state);
                set_agent_type(Asset);
            }
    };


    class USVAgent : public BaseAgent
    {
        // private variables
        AgentAssignment current_assignment;
        public:
            
            void copy(const USVAgent &usv){
                BaseAgent::copy(usv);
                set_current_assignment(usv.get_current_assignment());
            }
            AgentAssignment get_current_assignment() const{
                return current_assignment;
            }

            void set_current_assignment(AgentAssignment assignment){
                current_assignment=assignment;
            }
            int switch_observe_to_delay_assignment(int intruder_id){
                if(has_delay_task()) return -1;
                bool had_observe_task = remove_observe_assignment(intruder_id);
                if(had_observe_task){
                    set_delay_assignment(intruder_id);
                    return 1;
                }else{
                    return 0;
                }
            }
            void set_delay_assignment(int delay_assignment_idx){
                for(auto &task : current_assignment){
                    if(task.task_type==Delay){
                        task.task_idx=delay_assignment_idx;
                        return;
                    } else if(task.task_type==Observe && task.task_idx==delay_assignment_idx){
                        task.task_idx=delay_assignment_idx;
                        task.task_type=Delay;
                        return;
                        
                    }
                }
                current_assignment.push_back(AgentTask(Delay, delay_assignment_idx));

            }
            void set_guard_assignment(int guard_assignment_idx){
                for(auto &task : current_assignment){
                    if(task.task_type==Guard){
                        task.task_idx=guard_assignment_idx;
                        return;
                    }
                }
                current_assignment.push_back(AgentTask(Guard, guard_assignment_idx));
            }
            
            void add_observe_task(int observe_assignment_idx){
                remove_observe_assignment(observe_assignment_idx);
                current_assignment.push_back(AgentTask(Observe, observe_assignment_idx));
            }
            bool has_delay_task(){
                for(const auto &task : current_assignment){
                    if(task.task_type==Delay && task.task_idx!=-1) return true;
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
            USVAgent(){}
            USVAgent(AgentState state,
                     AgentConstraints constraints,
                     CollisionAvoidanceParameters ca_params,
                     AgentAssignment assignment)
                    : BaseAgent(state, constraints, ca_params, USV)
                {
                    set_current_assignment(assignment);
                }

            USVAgent(AgentState state){
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
    };


    int get_left_and_right_points();
}
#endif