#include <iostream>
#include <math.h>
#include "ros/ros.h"
#ifndef SWARM_H
#define SWARM_H
namespace swarm_tools{
    const double PI = 3.14159265358979323846;
    struct Point2D{
        double x;
        double y;

        bool operator==(const Point2D& rhs) const;
        bool operator!=(const Point2D& rhs) const;
        Point2D operator-(const Point2D& rhs) const;
        Point2D operator+(const Point2D& rhs) const;
        Point2D operator*(const double c) const;
        //friend std::ostream& operator<<(std::ostream& stream, Point2D p);
    };
    struct Vector2D:Point2D{
        Vector2D(const Point2D& start, const Point2D& end);
        double dot(Point2D& rhs) const;
        double norm();
    };

    struct PointInterval{
        Point2D left_point;
        Point2D right_point;
    };

    struct AngleInterval{
        double l_theta_rad;
        double r_theta_rad;

        bool operator<(const AngleInterval& rhs) const;
        bool operator==(const AngleInterval& rhs) const;
        bool contains(double theta_rad) const;
    };


    double radnorm(double rad);
    double deg2rad(double rad);
    double rad2deg(double deg);

    bool greater_ai(const AngleInterval &ai1, const AngleInterval &ai2);

    double clip(const double& x,
                const double& min_x,
                const double& max_x);
    
    Point2D center_point(const Point2D& p1,
                         const Point2D& p2);

    Point2D mid_point(const Point2D& p1, 
                      const Point2D& p2);
    double absolute_angle_between_points(
        const Point2D& p1,
        const Point2D& p2
    );
    double relative_angle_between_points(
        const Point2D& p1,
        const Point2D& p2,
        const double& offset_angle);

    double euclidean_distance(
        const Point2D& p1,
        const Point2D& p2
    );
    double log_multivariate_normal_2d(double z_x, double z_y, double mu_x, double mu_y, double sigma ){
        double dx = z_x-mu_x;
        double dy = z_y-mu_y;

        double constant = -log(2*PI*sigma*sigma);
        double exponent = -0.5 *(dx*dx + dy*dy)/(sigma*sigma);
//        ROS_ERROR("exp: %f const: %f", exponent, constant);

        return exponent + constant;
    }

    double sigmoid(double a){
        return 1/(1+exp(-a));
    }

    int edge_points_of_circle(
        const Point2D& reference,
        const Point2D& center,
        const double& radius,
        const double& heading,
        Point2D& leftmost_point,
        Point2D& rightmost_point
    );
}
#endif