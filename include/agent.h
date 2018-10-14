
enum AgentType{
    USV,
    Intruder,
    Static,
    Asset
};

struct AgentState
{
    double x;
    double y;
    double speed;
    double heading;
    double radius;
    double sim_id;

    Point2D position() const;
};

struct AgentCommand
{
    double delta_speed;
    double delta_heading;
};

struct AgentConstraints
{
    const double max_speed;
    const double max_delta_speed;
    const double max_delta_heading; 
};

int get_left_and_right_points();