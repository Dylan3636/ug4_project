#define BOOST_TEST_MODULE COLLISION_AVOIDANCE

#include <iostream>
#include "agent.h"
#include <cmath>
#include <boost/test/unit_test.hpp>
#include "collision_avoidance.h"

using namespace swarm_tools;



BOOST_AUTO_TEST_CASE(collision_intervals_inside){

    double x = 10;
    double y = 20;
    double speed = 0;
    double heading = 0;

    agent::AgentState agent_state = {x, y, speed, heading};

    double max_distance = 100;
    double max_angle_rad = PI/2;

    double l_theta_1 = PI/4;
    double r_theta_1 = -PI/6;
    double l_theta_2 = PI/6;
    double r_theta_2 = -PI/4;

    AngleInterval ai1 = {l_theta_1, r_theta_1};
    AngleInterval ai2 = {l_theta_2, r_theta_2};

    Point2D lp1 = {cos(l_theta_1) + x, sin(l_theta_1) + y};
    Point2D rp1 = {cos(r_theta_1) + x, sin(r_theta_1) + y};
    Point2D lp2 = {cos(l_theta_2) + x, sin(l_theta_2) + y};
    Point2D rp2 = {cos(r_theta_2) + x, sin(r_theta_2) + y};

    PointInterval pi1 = {lp1, rp1};
    PointInterval pi2 = {lp2, rp2};

    std::vector<PointInterval> edges;
    edges.push_back(pi1);
    edges.push_back(pi2);

    std::vector<AngleInterval> safe_intervals;

    int flag = collision_avoidance::get_safe_intervals(
        agent_state,
        edges,
        max_distance,
        max_angle_rad,
        safe_intervals
    );
    std::cout << flag <<std::endl;
    std::cout << safe_intervals.size() <<std::endl;
    for (AngleInterval si : safe_intervals){
        std::cout << "(" << si.l_theta_rad*(180/PI) << ", " << si.r_theta_rad*(180/PI) << ")"<<std::endl;
    }
    BOOST_CHECK_CLOSE(safe_intervals[0].l_theta_rad, PI/2, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[0].r_theta_rad, PI/4, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[1].l_theta_rad, -PI/4, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[1].r_theta_rad, -PI/2, 1e-10);
}

BOOST_AUTO_TEST_CASE(collision_intervals_outside){

    double x = 10;
    double y = 20;
    double speed = 0;
    double heading = 0;

    agent::AgentState agent_state = {x, y, speed, heading};

    double max_distance = 100;
    double max_angle_rad = PI/2;

    double l_theta_1 = PI/4;
    double r_theta_1 = PI/6;
    double l_theta_2 = -PI/6;
    double r_theta_2 = -PI/4;

    AngleInterval ai1 = {l_theta_1, r_theta_1};
    AngleInterval ai2 = {l_theta_2, r_theta_2};

    Point2D lp1 = {cos(l_theta_1) + x, sin(l_theta_1) + y};
    Point2D rp1 = {cos(r_theta_1) + x, sin(r_theta_1) + y};
    Point2D lp2 = {cos(l_theta_2) + x, sin(l_theta_2) + y};
    Point2D rp2 = {cos(r_theta_2) + x, sin(r_theta_2) + y};

    PointInterval pi1 = {lp1, rp1};
    PointInterval pi2 = {lp2, rp2};

    std::vector<PointInterval> edges;
    edges.push_back(pi1);
    edges.push_back(pi2);

    std::vector<AngleInterval> safe_intervals;

    int flag = collision_avoidance::get_safe_intervals(
        agent_state,
        edges,
        max_distance,
        max_angle_rad,
        safe_intervals
    );
    std::cout << flag <<std::endl;
    std::cout << safe_intervals.size() <<std::endl;
    for (AngleInterval si : safe_intervals){
        std::cout << "(" << si.l_theta_rad*(180/PI) << ", " << si.r_theta_rad*(180/PI) << ")"<<std::endl;
    }
    BOOST_CHECK_CLOSE(safe_intervals[0].l_theta_rad, PI/2, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[0].r_theta_rad, PI/4, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[1].l_theta_rad, PI/6, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[1].r_theta_rad, -PI/6, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[2].l_theta_rad, -PI/4, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[2].r_theta_rad, -PI/2, 1e-10);
}

BOOST_AUTO_TEST_CASE(collision_intervals_outside_left){

    double x = 10;
    double y = 20;
    double speed = 0;
    double heading = 0;

    agent::AgentState agent_state = {x, y, speed, heading};

    double max_distance = 100;
    double max_angle_rad = PI/2;

    double l_theta_1 = -PI/6;
    double r_theta_1 = -PI/4;
    double l_theta_2 = PI/4;
    double r_theta_2 = PI/6;

    AngleInterval ai1 = {l_theta_1, r_theta_1};
    AngleInterval ai2 = {l_theta_2, r_theta_2};

    Point2D lp1 = {cos(l_theta_1) + x, sin(l_theta_1) + y};
    Point2D rp1 = {cos(r_theta_1) + x, sin(r_theta_1) + y};
    Point2D lp2 = {cos(l_theta_2) + x, sin(l_theta_2) + y};
    Point2D rp2 = {cos(r_theta_2) + x, sin(r_theta_2) + y};

    PointInterval pi1 = {lp1, rp1};
    PointInterval pi2 = {lp2, rp2};

    std::vector<PointInterval> edges;
    edges.push_back(pi1);
    edges.push_back(pi2);

    std::vector<AngleInterval> safe_intervals;

    int flag = collision_avoidance::get_safe_intervals(
        agent_state,
        edges,
        max_distance,
        max_angle_rad,
        safe_intervals
    );
    std::cout << flag <<std::endl;
    std::cout << safe_intervals.size() <<std::endl;
    for (AngleInterval si : safe_intervals){
        std::cout << "(" << si.l_theta_rad*(180/PI) << ", " << si.r_theta_rad*(180/PI) << ")"<<std::endl;
    }
    BOOST_CHECK_CLOSE(safe_intervals[0].l_theta_rad, PI/2, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[0].r_theta_rad, PI/4, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[1].l_theta_rad, PI/6, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[1].r_theta_rad, -PI/6, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[2].l_theta_rad, -PI/4, 1e-10);
    BOOST_CHECK_CLOSE(safe_intervals[2].r_theta_rad, -PI/2, 1e-10);
}

BOOST_AUTO_TEST_CASE(collision_check_test){
    double x = 10;
    double y = 20;
    double speed = 0;
    double heading = 0;
    double max_distance = 100;
    double max_angle_rad = PI/2;
    double l_theta_1 = PI/4;
    double r_theta_1 = -PI/4;

    agent::AgentState agent_state = {x, y, speed, heading};


    AngleInterval ai1 = {l_theta_1, r_theta_1};

    Point2D lp = {cos(l_theta_1) + x, sin(l_theta_1) + y};
    Point2D rp = {cos(r_theta_1) + x, sin(r_theta_1) + y};

    double l_theta_col;
    double r_theta_col;

    bool flag = collision_avoidance::collision_check(
        agent_state,
        lp,
        rp,
        max_distance,
        max_angle_rad,
        l_theta_col,
        r_theta_col
    );
    // std::cout << "(" << l_theta_col*(180/PI) << ", " << r_theta_col*(180/PI) << ")";
    BOOST_CHECK(flag);
    BOOST_CHECK_CLOSE(l_theta_1, l_theta_col, 1e-10);
    BOOST_CHECK_CLOSE(r_theta_1, r_theta_col, 1e-10);
}
BOOST_AUTO_TEST_CASE(collision_check_test_left_edge){
    double x = 10;
    double y = 20;
    double speed = 0;
    double heading = 0;
    double max_distance = 100;
    double max_angle_rad = PI/2;
    double l_theta_1 = PI;
    double r_theta_1 = -PI/4;

    agent::AgentState agent_state = {x, y, speed, heading};


    AngleInterval ai1 = {l_theta_1, r_theta_1};

    Point2D lp = {cos(l_theta_1) + x, sin(l_theta_1) + y};
    Point2D rp = {cos(r_theta_1) + x, sin(r_theta_1) + y};

    double l_theta_col;
    double r_theta_col;

    bool flag = collision_avoidance::collision_check(
        agent_state,
        lp,
        rp,
        max_distance,
        max_angle_rad,
        l_theta_col,
        r_theta_col
    );
    // std::cout << "(" << l_theta_col*(180/PI) << ", " << r_theta_col*(180/PI) << ")";
    BOOST_CHECK(flag);
    BOOST_CHECK_CLOSE(PI/2, l_theta_col, 1e-10);
    BOOST_CHECK_CLOSE(-PI/4, r_theta_col, 1e-10);
}
BOOST_AUTO_TEST_CASE(collision_check_test_right_edge){
    double x = 10;
    double y = 20;
    double speed = 0;
    double heading = 0;
    double max_distance = 100;
    double max_angle_rad = PI/2;
    double l_theta_1 = PI/4;
    double r_theta_1 = -PI+0.5;

    agent::AgentState agent_state = {x, y, speed, heading};


    AngleInterval ai1 = {l_theta_1, r_theta_1};

    Point2D lp = {cos(l_theta_1) + x, sin(l_theta_1) + y};
    Point2D rp = {cos(r_theta_1) + x, sin(r_theta_1) + y};

    double l_theta_col;
    double r_theta_col;

    bool flag = collision_avoidance::collision_check(
        agent_state,
        lp,
        rp,
        max_distance,
        max_angle_rad,
        l_theta_col,
        r_theta_col
    );

    // std::cout << "(" << flag;
    // std::cout << "(" << l_theta_col*(180/PI) << ", " << r_theta_col*(180/PI) << ")";
    BOOST_CHECK(flag);
    BOOST_CHECK_CLOSE(PI/4, l_theta_col, 1e-10);
    BOOST_CHECK_CLOSE(-PI/2, r_theta_col, 1e-10);
}

BOOST_AUTO_TEST_CASE(collision_check_test_right_edge){
    double x = 10;
    double y = 20;
    double speed = 0;
    double heading = 0;
    double max_distance = 100;
    double max_angle_rad = PI/2;
    double l_theta_1 = PI/4;
    double r_theta_1 = -PI+0.5;

    agent::AgentState agent_state = {x, y, speed, heading};


    AngleInterval ai1 = {l_theta_1, r_theta_1};

    Point2D lp = {cos(l_theta_1) + x, sin(l_theta_1) + y};
    Point2D rp = {cos(r_theta_1) + x, sin(r_theta_1) + y};

    double l_theta_col;
    double r_theta_col;

    bool flag = collision_avoidance::correct_command(
        agent_state,
        lp,
        rp,
        max_distance,
        max_angle_rad,
        l_theta_col,
        r_theta_col
    );

    // std::cout << "(" << flag;
    // std::cout << "(" << l_theta_col*(180/PI) << ", " << r_theta_col*(180/PI) << ")";
    BOOST_CHECK(flag);
    BOOST_CHECK_CLOSE(PI/4, l_theta_col, 1e-10);
    BOOST_CHECK_CLOSE(-PI/2, r_theta_col, 1e-10);
}

BOOST_AUTO_TEST_CASE(in_fan_test){
        double distance = 10;
        double l_theta_rad = PI/4;
        double r_theta_rad = -PI/4;
        double max_distance = 100;


    }