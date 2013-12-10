#include "djikstra.hpp"

namespace nslo
{

void clear_paths()
{
    robot_list::iterator r_iter;

    for (r_iter = robots.begin(); r_iter != robots.end(); ++r_iter)
    {
        (*r_iter)->path.clear();
        (*r_iter)->orientation.x = 0.0;
        (*r_iter)->orientation.y = 0.0;
    }
}

void print_path(int source, int node, int* predecessor, Robot* robot)
{
    if (node == source)
    {
        robot->path.push_back(randPoints[source]);
        std::cout << (char)(node + 97) << "..";
    }
    else if (predecessor[node] == -1)
    {
        std::cout << "No path from “<<source<<” to "<< (char)(node + 97) << std::endl;
        clear_paths();
        return;
    }
    else
    {
        print_path(source, predecessor[node], predecessor, robot);
        robot->path.push_back(randPoints[node]);
        std::cout << (char) (node + 97) << "..";
    }

    replan = false;
}

void dijkstra(int source, int goal, Robot* robot)
{
    // initialize; the distance between source to source is zero and all other
    // distances between source and vertices are infinity. The mark is initialized
    // to false and predecessor is initialized to -1
    int predecessor[PRM_POINTS];
    double distance[PRM_POINTS];
    bool mark[PRM_POINTS]; //keep track of visited node
    //double minDistance = infinity;
    int closestUnmarkedNode;
    int count = 0;

    for (int i = 0; i < PRM_POINTS; i++)
    {
        mark[i] = false;
        predecessor[i] = -1;
        distance[i] = infinity;
    }

    distance[source] = 0;

    while (count < PRM_POINTS)
    {
        // return the node which is nearest from the Predecessor marked node.
        // If the node is already marked as visited, then it search for another
        int minDistance = infinity;

        for (int i = 0; i < PRM_POINTS; i++)
        {
            if((!mark[i]) && (minDistance >= distance[i]))
            {
                minDistance = distance[i];
                closestUnmarkedNode = i;
            }
        }

        mark[closestUnmarkedNode] = true;

        for (int i = 0; i < PRM_POINTS; i++)
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

    print_path(source, goal, predecessor, robot);
    std::cout << "->" << distance[goal] << std::endl;
}

}
