#define BOOST_TEST_MAIN
#include <swarm_tools.h>
#include <algorithm>

namespace swarm_tools{

    bool Point2D::operator==(const Point2D& rhs) const {
        return (this->x == rhs.x) && (this->y == rhs.y);
    }
    bool Point2D::operator!=(const Point2D& rhs) const {
        return !(*this == rhs);
    }
    Point2D Point2D::operator-(const Point2D& rhs) const {
        return Point2D{this->x - rhs.x, this->y - rhs.y};
    }
    Point2D Point2D::operator+(const Point2D& rhs) const {
        return Point2D{this->x + rhs.x, this->y + rhs.y};
    }
    Point2D Point2D::operator*(const double c) const {
        return Point2D{this->x*c, this->y*c};
    }
    std::ostream& operator << (std::ostream& stream, Point2D& p){
        return stream;
    }
    Vector2D::Vector2D(const Point2D& start, const Point2D& end) {
        Point2D diff = start-end;
        this->x = diff.x;
        this->y = diff.y;
    }

    double Vector2D::dot(Point2D& rhs) const{
        return  this->x * rhs.x + this->y * rhs.y;
    }

    double Vector2D::norm(){
        return  std::sqrt(this->x * this->x + this->y * this->y);
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
        std::cout << "Contains: "<< l_theta_rad <<", "<< angle_rad << ", " << r_theta_rad << std::endl;
        return this->r_theta_rad<=angle_rad &&  angle_rad <= this->l_theta_rad;
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
        if (left > swarm_tools::PI){left -= 2*swarm_tools::PI;}
        if (left < -swarm_tools::PI){left += 2*swarm_tools::PI;}
        return left;
    }
    double euclidean_distance(
        const Point2D& p1,
        const Point2D& p2
    ){
        std::cout<< "Point 1 : (" << p1.x << ", " << p1.y << ")" << std::endl;
        std::cout<< "Point 2 : (" << p2.x << ", " << p2.y << ")" << std::endl;
        double dx = p1.x-p2.x;
        double dy = p1.y-p2.y;
        return sqrt(dx*dx + dy*dy);
    }
    int edge_points_of_circle(
        const Point2D& reference,
        const Point2D& center,
        const double& radius,
        const double& heading,
        Point2D& leftmost_point,
        Point2D& rightmost_point
    ){
        double r = euclidean_distance(reference, center);
        double alpha;

        alpha = atan2(radius, r);

        Point2D centered_point = center_point(reference, center);
        double beta = atan2(centered_point.y, centered_point.x);
        double hyp = sqrt(r*r + radius*radius);

        std::cout << "Alpha: " << alpha*180/PI << std::endl;
        std::cout << "Beta: " << beta*180/PI << std::endl;
        std::cout << "Heading: " << heading*180/PI << std::endl;

        leftmost_point = {reference.x + hyp*cos(alpha + beta), 
                          reference.y + hyp*sin(alpha + beta)};
        rightmost_point = {reference.x + hyp*cos(-alpha + beta), 
                           reference.y + hyp*sin(-alpha + beta)};
        std::cout<< "Leftmost Point : (" << leftmost_point.x << ", " << leftmost_point.y << ")" << std::endl;
        std::cout<< "Rightmost Point : (" << rightmost_point.x << ", " << rightmost_point.y << ")" << std::endl;
        return -1;
    }
}