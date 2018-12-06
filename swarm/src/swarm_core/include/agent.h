#include "swarm_tools.h"
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

    struct AgentAssignment
    {
        int delay_assignment_idx;
        int guard_assignment_idx;
    };

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

            float get_x() const{
                return state.x;
            }

            float get_y() const{
                return state.y;
            }

            float get_speed() const{
                return state.speed;
            }

            float get_heading() const{
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

            AgentState project_state_forward(int timesteps,
                                            double delta_time_secs
                                            );

            void command_agent_forward(const AgentCommand &command,
                                            double delta_time_secs){

                double x = this->state.x;
                double y = this->state.y;
                double speed = this->state.speed;
                double heading = this->state.heading;

                speed += std::min(command.delta_speed*delta_time_secs, this->constraints.max_speed);
                heading += fmod(command.delta_heading*delta_time_secs, 2*swarm_tools::PI);

                x += speed*cos(heading);
                y += speed*sin(heading); 
                set_state(AgentState(x, y, speed, heading, state.radius, state.sim_id));
        }
    };

    class IntruderAgent : public BaseAgent
    {   
        public:
            IntruderAgent(){}
            IntruderAgent(AgentState state,
                        AgentConstraints constraints,
                        CollisionAvoidanceParameters ca_params)
                    : BaseAgent(state, constraints, ca_params, Intruder){}
            IntruderAgent(const AgentState &state){
                set_state(state);
                set_constraints(AgentConstraints{30, 5, swarm_tools::PI/2});
                set_collision_avoidance_params(CollisionAvoidanceParameters{100, swarm_tools::PI, 0.5});
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
            AgentAssignment get_current_assignment() const;
            void set_current_assignment(AgentAssignment assignment);

            void copy(const USVAgent &usv);
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
                set_constraints(AgentConstraints{35,
                                                5,
                                                swarm_tools::PI/2});
                set_collision_avoidance_params(
                    CollisionAvoidanceParameters{100,
                                                swarm_tools::PI,
                                                0.9});
                int delay_id = 101;
                int guard_id = 0;
                set_current_assignment(AgentAssignment {delay_id, guard_id});
                set_agent_type(USV);
            }
    };


    int get_left_and_right_points();
}
#endif