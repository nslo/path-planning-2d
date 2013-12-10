#pragma once

#include "geometry.hpp"

namespace nslo
{

class Circle : public Geometry
{
public:
    // Constructor and destructor
    Circle(const Vector2& _center, const double& _radius);
    virtual ~Circle();

    // Member variables
    double radius;

    // Helper functions
    int get_num_circle_segments(double r);
    void calculate_points();

    // Drawing
    virtual void draw(); 
    virtual bool on_down(Vector2 p); 
    virtual void on_drag(Vector2 p);
    virtual void move(double dx, double dy);

    // Geometry
    virtual bool point_in_polygon(Vector2 p);
    virtual bool edge_polygon_intersect(Vector2 e1, Vector2 e2);
    virtual Vector2 get_centroid();
};

}
