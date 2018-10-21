#define BOOST_TEST_MAIN
#include <swarm_tools.h>
#include <math.h>
#include <algorithm>
#include <iostream>

namespace swarm_tools{

    bool Point2D::operator==(const Point2D& rhs) const {
        return (this->x == rhs.x) && (this->y == rhs.y);
    }

    bool Point2D::operator!=(const Point2D& rhs) const {
        return !(*this == rhs);
    }
    std::ostream& operator << (std::ostream& stream, Point2D& p){
        stream << " (" << p.x << ", " << p.y << ")";
        return stream;
    }

    bool AngleInterval::operator<(const AngleInterval& rhs) const{
         return this->l_theta_rad < rhs.l_theta_rad;
     }
    bool AngleInterval::operator==(const AngleInterval& rhs) const {
        return (this->l_theta_rad == rhs.l_theta_rad) && (this->r_theta_rad == rhs.r_theta_rad);
    }

    bool greater_ai(const AngleInterval &ai1, const AngleInterval &ai2){
         return ai1.l_theta_rad > ai2.l_theta_rad;
    }

    bool AngleInterval::contains(const double angle_rad)const{
        return this->r_theta_rad<angle_rad &&  angle_rad < this->l_theta_rad;
    }
    

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

    double absolute_angle_between_points(
        const Point2D& p1,
        const Point2D& p2
    ){
        Point2D centered_p2 = center_point(p1, p2);
        std::cout << "(" << centered_p2.x << ", " << centered_p2.y << ")" << std::endl;
        double angle = atan2(centered_p2.y, centered_p2.x);
        return angle;
    }

    double relative_angle_between_points(
        const Point2D& p1,
        const Point2D& p2,
        const double& offset_angle
    ){
        double abs_angle = swarm_tools::absolute_angle_between_points(p1,
                                                                      p2);
        if (abs_angle<0){
            abs_angle += 2*swarm_tools::PI;
        }
        double rel_angle;
        double left = abs_angle-offset_angle;
        if (std::abs(left) <= swarm_tools::PI)
            {rel_angle = left;}
        else if (left<0)
            {rel_angle = left + 2*swarm_tools::PI;}
        else
            {rel_angle = left -2*swarm_tools::PI;}

        return rel_angle;
    }
    double euclidean_distance(
        const Point2D& p1,
        const Point2D& p2
    ){
        double dx = p1.x-p2.x;
        double dy = p1.y-p2.y;
        return sqrt(dx*dx + dy*dy);
    }
    int edge_points_of_circle(
        const Point2D& reference,
        const Point2D& center,
        const double& radius,
        Point2D& leftmost_point,
        Point2D& rightmost_point
    ){
        double r = euclidean_distance(reference, center);
        double alpha = atan2(radius, r);
        Point2D centered_point = center_point(reference, center);
        double beta = atan2(centered_point.y, centered_point.x);
        double hyp = sqrt(r*r + radius*radius);
        leftmost_point = {hyp*cos(beta + alpha) + reference.x,
                          hyp*sin(beta + alpha) + reference.y};
        rightmost_point = {hyp*cos(beta - alpha) - reference.x,
                          hyp*sin(beta - alpha) - reference.y};
        return -1;
    }
}