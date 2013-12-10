#pragma once

#include "geometry.hpp"

namespace nslo
{

class Triangle : public Geometry
{
public:
    // Constructor and Destructor
    Triangle(std::vector<Vector2> _pts);
    virtual ~Triangle();

    // Member variables
    double radius; // "radius" of the triangle
    double get_radius();

    // Drawing
    virtual void draw(); 
    virtual bool on_down(Vector2 p); // call on mouse-down
    virtual void on_drag(Vector2 p); // call on mouse drag to move the corner
    virtual void move(double dx, double dy);

    // Geometry
    virtual Vector2 get_centroid();
    virtual bool point_in_polygon(Vector2 p);
    virtual bool edge_polygon_intersect(Vector2 e1, Vector2 e2);
};

}
