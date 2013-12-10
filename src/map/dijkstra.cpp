#include "dijkstra.hpp"

namespace nslo
{

void clear_paths(std::vector<Robot *> robots)
{
    for (auto r_iter = robots.begin(); r_iter != robots.end(); ++r_iter)
    {
        (*r_iter)->path.clear();
        (*r_iter)->orientation.x = 0.0;
        (*r_iter)->orientation.y = 0.0;
    }
}

void print_path(int source, int node, std::vector<int> predecessor, Robot* robot,
        std::vector<Vector2> rand_points, bool& replan, std::vector<Robot *> robots)
{
    if (node == source)
    {
        robot->path.push_back(rand_points[source]);
        std::cout << (char)(node + 97) << "..";
    }
    else if (predecessor[node] == -1)
    {
        std::cout << "No path from “<<source<<” to "<< (char)(node + 97) << std::endl;
        clear_paths(robots);
        return;
    }
    else
    {
        print_path(source, predecessor[node], predecessor, robot, rand_points, replan, robots);
        robot->path.push_back(rand_points[node]);
        std::cout << (char) (node + 97) << "..";
    }

    replan = false;
}

void dijkstra(int source, int goal, Robot* robot, size_t num_vertices,
       std::vector<Vector2> rand_points, bool& replan, std::vector<Robot *> robots)
{
    // initialize; the distance between source to source is zero and all other
    // distances between source and vertices are infinity. The mark is initialized
    // to false and predecessor is initialized to -1
    std::vector<int> predecessor;
    std::vector<double> distance;
    std::vector<bool> mark; //keep track of visited node
    int closestUnmarkedNode;
    size_t count = 0;

    for (size_t i = 0; i < num_vertices; i++)
    {
        mark.push_back(false);
        predecessor.push_back(-1);
        distance.push_back(infinity);
    }

    std::cout << num_vertices << std::endl;
    assert(predecessor.size() == num_vertices);
    assert(distance.size() == num_vertices);
    assert(mark.size() == num_vertices);

    distance[source] = 0;

    while (count < num_vertices)
    {
        // return the node which is nearest from the Predecessor marked node.
        // If the node is already marked as visited, then it search for another
        double minDistance = infinity;

        for (size_t i = 0; i < num_vertices; i++)
        {
            if((!mark[i]) && (minDistance >= distance[i]))
            {
                minDistance = distance[i];
                closestUnmarkedNode = i;
            }
        }

        mark[closestUnmarkedNode] = true;

        for (size_t i = 0; i < num_vertices; i++)
        {
            if ((!mark[i]) && (adjacency[closestUnmarkedNode][i] > 0) )
            {
                if (distance[i] > distance[closestUnmarkedNode] +
                        adjacency[closestUnmarkedNode][i])
                {
                    distance[i] = distance[closestUnmarkedNode] +
                        adjacency[closestUnmarkedNode][i];
                    predecessor[i] = closestUnmarkedNode;
                }
            }
        }

        count++;
    }

    print_path(source, goal, predecessor, robot, rand_points, replan, robots);

    std::cout << "->" << distance[goal] << std::endl;
}

}
