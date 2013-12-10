#pragma once

#include <vector>
#include "../draw/drawable.hpp"

namespace nslo
{

bool close_enough(Vector2 a, Vector2 b);

bool IsOnSegment(double xi, double yi, double xj, double yj,
        double xk, double yk);

char ComputeDirection(double xi, double yi, double xj, double yj,
        double xk, double yk);

/** Do line segments (x1, y1)--(x2, y2) and (x3, y3)--(x4, y4) intersect? */
bool edge_edge_intersect(double x1, double y1, double x2, double y2,
        double x3, double y3, double x4, double y4);
 
class Geometry : public Drawable
{
public: 
    // Constructor and Destructor
    Geometry();
    virtual ~Geometry();

    // Member variables
    size_t num_points;
    std::vector<Vector2> pts; // the points defining the polygon
    Vector2 centroid;

    // Geometry
    virtual Vector2 get_centroid() = 0;
    virtual bool point_in_polygon(Vector2 p) = 0;
    virtual bool edge_polygon_intersect(Vector2 e1, Vector2 e2) = 0;
};

}
