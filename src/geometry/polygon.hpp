#pragma once

#include "geometry.hpp"

namespace nslo
{

class Polygon : public Geometry
{
public:
    // Constructor and destructor
    Polygon(const std::vector<Vector2>& _pts);
    virtual ~Polygon();

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
