#pragma once

#include "vector.hpp"

namespace nslo {
    bool close_enough(Vector2 a, Vector2 b);
    bool point_in_triangle(Vector2 p, Vector2* pts);
    bool point_in_polygon(Vector2 p, Vector2* pts, int n);
    bool edge_polygon_intersection(Vector2 e1, Vector2 e2, Vector2* pts, int n);
}

