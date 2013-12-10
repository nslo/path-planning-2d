#include <iostream>
#include <vector>
#include "draw/drawable.hpp"
#include "draw/draw.hpp"
#include "map/map.hpp"
#include "map/prm.hpp"

#define UNUSED(expr) do { (void)(expr); } while (0)

using namespace nslo;

//void init(std::vector<Robot *>& robots,
//        std::vector<Obstacle *>& obstacles,
//        std::vector<Goal *>& goals, std::vector<Vector2>& rand_points);
void init();
void reset();
void step();
void idle();

namespace nslo 
{

// Global variables
std::vector<Robot *> robots; // the list of our robots
std::vector<Obstacle *> obstacles; // the list of our obstacles
std::vector<Goal *> goals; // the list of our obstacles
Robot *active_robot = nullptr; // the active triangle while dragging
Obstacle *active_obstacle = nullptr; // the active hexagon while dragging
Goal *active_goal = nullptr; // the active hexagon while dragging
bool SHOW_POINTS = true;
size_t PRM_POINTS = 100;
double infinity = std::numeric_limits<double>::max();
std::vector<Vector2> rand_points;
bool paused;
bool replan = true;
int windowWidth, windowHeight; // dimensions of the window in pixel
std::vector<std::vector<double>> adjacency;

}

int main(int argc, char *argv[])
{

    // Glut stuff
    glutInit(&argc, argv); // init GLUT library
    glutInitDisplayMode(GLUT_SINGLE|GLUT_RGB); // true color and single buffer
    glutInitWindowSize(1600, 1024); // set initial window size
    glutCreateWindow("Float"); // open window with a title

    // Initialize objects
    init();
    //init(robots, obstacles, goals, rand_points);

    // Register callback functions for redraw, mouse events etc.
    glutDisplayFunc(display);
    glutMouseFunc(mouse_func);
    glutMotionFunc(drag);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard_func);

    // Animation and stuff goes here
    glutIdleFunc(idle);

    // Create right button popup menu
    glutCreateMenu(menu_func);
    glutAddMenuEntry("New Robot", 1);
    glutAddMenuEntry("New Obstacle", 2);
    glutAddMenuEntry("New Goal", 3);
    glutAddMenuEntry("Quit", 4);
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    // Enter main loop
    glutMainLoop();

    return 0;
}

//void init(std::vector<Robot *>& robots,
//        std::vector<Obstacle *>& obstacles,
//        std::vector<Goal *>& goals, std::vector<Vector2>& rand_points)
void init()
{
    paused = true;
    replan = true;
    // Initial objects
    Vector2 center;
    double radius;
    get_new_robot(center, radius);
    robots.push_back(new Robot(center, radius));
    std::vector<Vector2> obstacle_pts;
    get_new_obstacle(obstacle_pts);
    obstacles.push_back(new Obstacle(obstacle_pts));
    std::vector<Vector2> goal_pts;
    get_new_goal(goal_pts);
    goals.push_back(new Goal(goal_pts));

    for (size_t i = 0; i < PRM_POINTS; i++)
    {
        rand_points.push_back(Vector2(0.0, 0.0));
    }

    for (size_t i = 0; i < PRM_POINTS; i++)
    {
        std::vector<double> row;
        adjacency.push_back(row);
        for (size_t j = 0; j < PRM_POINTS; j++)
        {
            adjacency[i].push_back(infinity);
        }
    }

    // init some OpenGL state
    glClearColor(0.1, 0.1, 0.1, 1.0); // grey background
    glShadeModel(GL_FLAT);
}

void reset()
{
    // clear polygon lists
    robots.clear();
    obstacles.clear();
    goals.clear();
    init();
    //init(robots, obstacles, goals, rand_points);
}

void step()
{
    for (auto i = robots.begin(); i != robots.end(); ++i)
    {
        double dx, dy;

        // first check to see if the path queue is empty (reached goal)
        if (!(*i)->path.empty())
        {

            ///////////////////////
            // quick check for clear path to goal
            // TODO not sure this should be necessary if prm/dijkstra working
            // - already looking for clear path to goal before making the plan
            // - but the closest point to the goal might not actually be that close...?
            // - consider
            bool goalBlocked = false;
            Vector2 tPos = (*i)->body.centroid;
            Vector2 gPos = goals.front()->body.centroid;

            for (auto h = obstacles.begin(); h != obstacles.end(); ++h)
            {
                goalBlocked |= (*h)->body.edge_polygon_intersect(tPos, gPos);
            }

            if (!goalBlocked)
            {
                (*i)->path.clear();
                (*i)->orientation.x = 0.0;
                (*i)->orientation.y = 0.0;
                Vector2 q = goals.front()->body.centroid;
                (*i)->path.push_back(q);
            }

            ///////////////////////





            Vector2 waypoint = (*i)->path.front();
            Vector2 c = (*i)->body.centroid;

            // check to see if orientation has been set yet
            if ((*i)->orientation.x == 0.0 && (*i)->orientation.y == 0.0)
            {
                Vector2 o = Vector2(waypoint.x - c.x, waypoint.y - c.y);
                (*i)->orientation = normalize(o);
            }

            // check to see if current destination is within EPSILON of
            // next coordinate in path queue; if so, pop
            // TODO turn
            if (std::abs(c.y - waypoint.y) < EPSILON &&
                    (std::abs(c.x - waypoint.x) < EPSILON))
            {
                (*i)->path.pop_front();

                if (!(*i)->path.empty())
                {
                    waypoint = (*i)->path.front();
                    Vector2 o = Vector2(waypoint.x - c.x, waypoint.y - c.y);
                    (*i)->orientation = normalize(o);
                }
            }
            // otherwise, move toward next waypoint
            else
            {
                double DELTA = 0.0002;
                dx = DELTA * (*i)->orientation.x;
                dy = DELTA * (*i)->orientation.y;
                (*i)->move(dx, dy);
            }
        }
    }

    glutPostRedisplay();
}

void idle()
{
    if (!paused)
    {
        // TODO robot proximity detection
        // TODO replan can be a (bit) vector?
        while (replan)
        {
            prm(robots, obstacles, goals, rand_points);

            bool validBounds = true;
            // TODO there's an error somewhere, maybe in when replan is set to false

            // for each triangle, find the closest point on the graph to triangle's current position
            // find closest point on graph to goal position
            // run dijkstra with those two points
            // push the resultant points onto the triangle's path
            // push goal onto the path

            for (auto r_iter = robots.begin(); r_iter != robots.end(); ++r_iter)
            {
                if (!validBounds)
                {
                    break;
                }

                Vector2 tPos = (*r_iter)->body.centroid;
                Vector2 gPos = goals.front()->body.centroid;

                ///////////////////////
                // quick check for clear path to goal
                /*
                bool goalBlocked = false;
                obstacle_list::iterator h;

                for (h = obstacles.begin(); h != obstacles.end(); ++h)
                {
                    goalBlocked |= edge_polygon_intersect(tPos.x, tPos.y,
                            gPos.x, gPos.y,
                            (*h)->pts, (*h)->num_points);
                }

                if (!goalBlocked)
                {
                    // TODO fix this for muli robots
                    replan = false;
                    break;
                }
                */
                ///////////////////////


                // check to see if closest nodes to start and goal are valid
                int tClosest = -1;
                int gClosest = -1;
                double tDist = infinity;
                double gDist = infinity;

                for (size_t i = 0; i < PRM_POINTS; i++)
                {
                    bool intersect1 = false, intersect2 = false;
                    for (auto h = obstacles.begin(); h != obstacles.end(); ++h)
                    {
                        intersect1 = (*h)->body.edge_polygon_intersect(tPos, rand_points[i]);

                        if (intersect1)
                        {
                            break;
                        }

                        intersect2 = (*h)->body.edge_polygon_intersect(gPos, rand_points[i]);

                        if (intersect2)
                        {
                            break;
                        }
                    }

                    if (intersect1 || intersect2)
                    {
                        continue;
                    }

                    double tempT = distance(tPos, rand_points[i]);
                    double tempG = distance(gPos, rand_points[i]);

                    if (tempT < tDist)
                    {
                        tClosest = i;
                        tDist = tempT;
                    }

                    if (tempG < gDist)
                    {
                        gClosest = i;
                        gDist = tempG;
                    }
                }

                if (tClosest == -1 || gClosest == -1)
                {
                    validBounds = false;
                    break;
                }

                dijkstra(tClosest, gClosest, *r_iter, PRM_POINTS,
                        rand_points, replan, robots);
            }
        }

        for (auto r_iter = robots.begin(); r_iter != robots.end(); ++r_iter)
        {
            Vector2 q = goals.front()->body.centroid;
            (*r_iter)->path.push_back(q);
        }

        step();
    }
}

