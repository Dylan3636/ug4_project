namespace swarm_tools{
    const double PI;

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

        bool contains(double theta_rad) const;
    };

    double clip(const double& x,
                const double& min_x,
                const double& max_x);
    
    Point2D center_point(const Point2D& p1,
                         const Point2D& p2);

    Point2D mid_point(const Point2D& p1, 
                      const Point2D& p2);

    double relative_angle_between_points(
        const Point2D& p1,
        const Point2D& p2,
        const double& offset_angle);
}