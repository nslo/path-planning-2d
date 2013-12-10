#pragma once

#include <limits>
#include "../math/vector.hpp"
#include "map.hpp"

namespace nslo
{

// Global variables

void prm(std::vector<Robot *> robots,
        std::vector<Obstacle *> obstacles,
        std::vector<Goal *> goals,
        std::vector<Vector2> rand_points);

}
