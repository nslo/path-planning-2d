#pragma once

namespace nslo
{

void clear_paths();

void print_path(int source, int node, int* predecessor, Robot* robot);

void dijkstra(int source, int goal, Robot* robot);

}
