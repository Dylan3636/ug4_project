#define BOOST_TEST_MODULE SWARM_TOOLS


#include <vector>
#include <algorithm>
#include <iostream>
#include <boost/test/unit_test.hpp>
#include "swarm_tools.h"

using namespace swarm_tools;

//#define BOOST_TEST_DONT_PRINT_LOG_VALUE
BOOST_AUTO_TEST_CASE (clip_test){
    BOOST_CHECK(clip(0, -10, 10) == 0);
    BOOST_CHECK(clip(20, -10, 10) == 10);
    BOOST_CHECK(clip(-20, -10, 10) == -10);
}

BOOST_AUTO_TEST_CASE(euclidean_distance_test){
    BOOST_CHECK_CLOSE(euclidean_distance(Point2D{3, 0}, Point2D{0, 4}), 5.0, 1e-16);
}

BOOST_AUTO_TEST_CASE(point_equality_test){
    Point2D p1 = {1, 1};
    Point2D p2 = {1, 1};
    BOOST_CHECK(p1==p2);
    Point2D p3 = {1.0, 1};
    BOOST_CHECK(p1==p3);
    Point2D p4 = {2, 1};
    BOOST_CHECK(p1!=p4);
    Point2D p5 = {1, 2};
    BOOST_CHECK(p1!=p5);
}

BOOST_AUTO_TEST_CASE(angle_interval_inequality_test){
    AngleInterval ai1 = {45, 30};
    AngleInterval ai2 = {30, 15};
    AngleInterval ai3 = {15, 0};
    BOOST_CHECK(ai2<ai1);
    BOOST_CHECK(ai3<ai1);
    BOOST_CHECK(ai3<ai2);
    BOOST_CHECK(!(ai1<ai1));
    BOOST_CHECK(!(ai1<ai2));
    BOOST_CHECK(!(ai2<ai3));
    BOOST_CHECK(!(ai1<ai3));
}

BOOST_AUTO_TEST_CASE(angle_interval_sort_test){
    AngleInterval ai1 = {45, 30};
    AngleInterval ai2 = {30, 15};
    AngleInterval ai3 = {15, 0};
    std::vector<AngleInterval> v;
    v.push_back(ai3);
    v.push_back(ai1);
    v.push_back(ai2);
    std::sort(v.begin(), v.end(), greater_ai);
    // std::cout << "("<< v[0].l_theta_rad<< ", " << v[0].r_theta_rad << ")" <<std::endl;
    BOOST_CHECK(v[0] == ai1);
    BOOST_CHECK(v[1] == ai2);
    BOOST_CHECK(v[2] == ai3);
}

BOOST_AUTO_TEST_CASE(mid_point_test){
    Point2D p1 = {1, 1};
    Point2D p2 = {3, 3};
    Point2D mid_pt = mid_point(p1, p2);
    Point2D pt = {2, 2};
    BOOST_CHECK(mid_pt== pt);
}

BOOST_AUTO_TEST_CASE(center_point_test){
    Point2D centered_pt = center_point(Point2D{1, 1}, Point2D{3, 3});
    Point2D pt = {2, 2};
    BOOST_CHECK(centered_pt == pt);
}