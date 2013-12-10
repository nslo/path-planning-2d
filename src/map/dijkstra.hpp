#pragma once

#include "map.hpp"
#include "../draw/draw.hpp"

namespace nslo
{

void clear_paths(std::vector<Robot *> robots);
void print_path(int source, int node, std::vector<int> predecessor, Robot* robot,
        std::vector<Vector2> rand_points, bool& replan, std::vector<Robot *> robots);
void dijkstra(int source, int goal, Robot* robot, size_t num_vertices,
       std::vector<Vector2> rand_points, bool& replan, std::vector<Robot *> robots);
}
