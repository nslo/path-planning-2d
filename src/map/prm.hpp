#pragma once

#include <limits>
#include "../math/vector.hpp"
#include "../map/map.hpp"

namespace nslo
{

// Global variables
const size_t PRM_POINTS = 100;
const int K = 4;
const double infinity = std::numeric_limits<double>::max();
double adjacency[PRM_POINTS][PRM_POINTS];

void prm(std::vector<Robot *> robots,
        std::vector<Obstacle *> obstacles,
        std::vector<Goal *> goals,
        std::vector<Vector2> rand_points);

}
