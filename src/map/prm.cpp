#include <vector>
#include "prm.hpp"

namespace nslo
{

void prm(std::vector<Robot *> robots,
        std::vector<Obstacle *> obstacles,
        std::vector<Goal *> goals, std::vector<Vector2> rand_points)
{
    // generate 100 random points from (0.0, 0.0) to (1.0, 1.0)
    size_t iter = 0;
    while (iter < PRM_POINTS)
    {
        bool valid = true;
        double x = (double) rand() / (RAND_MAX);
        double y = (double) rand() / (RAND_MAX);
        Vector2 tempPoint = Vector2(x, y);

        // disqualify points that:
        // a) are inside polygons
        // b) TODO are _radius_ distance from any line segment in any polygon
        // c) TODO are _radius_ distance from any vertex of any polygon

        for (auto j = obstacles.begin(); j != obstacles.end(); ++j)
        {
            if ((*j)->body.point_in_polygon(tempPoint))
            {
                valid = false;
                break;
            }
        }

        if (!valid)
        {
            break;
        }

        for (auto k = goals.begin(); k != goals.end(); ++k)
        {
            if ((*k)->body.point_in_polygon(tempPoint))
            {
                valid = false;
                break;
            }
        }

        if (!valid)
        {
            break;
        }

        for (auto i = robots.begin(); i != robots.end(); ++i)
        {
            if ((*i)->body.point_in_polygon(tempPoint))
            {
                valid = false;
                break;
            }
        }

        if (!valid)
        {
            break;
        }

        rand_points.push_back(tempPoint);
        iter++;
    }

    // turn sampled points into a graph
    // get the k-nearest-neighbors of each vertex
    // if the edge wouldn't cross an obstacle, make edge between them

    // TODO bounding boxes around polygons
    // TODO KNN (nlogn)

    // reset adjacency matrix
    for (size_t i = 0; i < PRM_POINTS; i++)
    {
        for (size_t j = 0; j < PRM_POINTS; j++)
        {
            adjacency[i][j] = infinity;
        }
    }

    for (size_t iter1 = 0; iter1 < PRM_POINTS; iter1++)
    {
        //int nearest[K];

        bool valid = true;

        //for (int iter2 = iter1 + 1; iter2 < PRM_POINTS; iter2++)
        for (size_t iter2 = iter1 + 1; iter2 < PRM_POINTS; iter2++)
        {
            for (auto j = obstacles.begin(); j != obstacles.end(); ++j)
            {
                bool intersect = (*j)->body.edge_polygon_intersect(rand_points[iter1],
                        rand_points[iter2]);

                if (intersect)
                {
                    adjacency[iter1][iter2] = infinity;
                    adjacency[iter2][iter1] = infinity;
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                break;
            }

            for (auto k = goals.begin(); k != goals.end(); ++k)
            {
                bool intersect = (*k)->body.edge_polygon_intersect(rand_points[iter1], 
                        rand_points[iter2]);

                if (intersect)
                {
                    adjacency[iter1][iter2] = infinity;
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                break;
            }

            for (auto i = robots.begin(); i != robots.end(); ++i)
            {
                bool intersect = (*i)->body.edge_polygon_intersect(rand_points[iter1],
                        rand_points[iter2]);

                if (intersect)
                {
                    adjacency[iter1][iter2] = infinity;
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                break;
            }

            double edgeDist = distance(rand_points[iter1], rand_points[iter2]);
            adjacency[iter1][iter2] = edgeDist;
            adjacency[iter2][iter1] = edgeDist;
        }
    }

    /*
    for (int a = 0; a < PRM_POINTS; a++)
    {
        for (int b = 0; b < PRM_POINTS; b++)
        {
            std::cout << adjacency[a][b] << "  ";
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;
    */




}

}
