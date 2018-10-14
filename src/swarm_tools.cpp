#include <swarm_tools.h>
#include <math.h>
#include <algorithm>

namespace swarm_tools{
    
    const double PI = 3.14159265358979323846;

    struct Point2D{
        double x;
        double y;
    }; 

    struct PointInterval{
        Point2D left_point;
        Point2D right_point;
    };

    struct AngleInterval{
        double l_theta_rad;
        double r_theta_rad;

        bool operator<(const AngleInterval& rhs) const{
            return this->l_theta_rad < rhs.l_theta_rad;
        }
    };

    double clip(const double& x, const double& min_x, const double& max_x){
        return std::min(max_x, std::max(min_x, x));
    }

    Point2D center_point(const Point2D& p1, const Point2D& p2){
        double x = p2.x - p1.x;
        double y = p2.y - p1.y;
        return Point2D{x, y};
    }

    Point2D mid_point(const Point2D& p1, const Point2D& p2){
        double x = 0.5*(p1.x+p2.x);
        double y = 0.5*(p1.y+p2.y);
        return Point2D{x, y};
    }

    double relative_angle_between_points(
        const Point2D& p1,
        const Point2D& p2,
        const double& offset_angle
    ){
        auto centered_p2 = center_point(p1, p2);
        double angle = atan2(centered_p2.y, centered_p2.x);
        if (angle<-PI/2){
            angle += 2*PI;
        }
        angle -= offset_angle;
        return angle;
    }
    double euclidean_distance(
        const Point2D& p1,
        const Point2D& p2
    ){
        double dx = p1.x-p2.x;
        double dy = p1.y-p2.y;
        return sqrt(dx*dx + dy*dy);
    }

}