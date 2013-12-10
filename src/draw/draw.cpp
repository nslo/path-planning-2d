#include "draw.hpp"
#include "../map/map.hpp"
#include <vector>

int windowWidth, windowHeight; // dimensions of the window in pixel
bool paused;
bool replan;
const double DELTA = 0.0002;
const bool SHOW_POINTS = true;

// global variables
typedef std::vector<Robot *> robot_list;
robot_list robots; // the list of our robots
Robot *active_robot = nullptr; // the active triangle while dragging

typedef std::vector<Obstacle *> obstacle_list;
obstacle_list obstacles; // the list of our obstacles
Obstacle *active_obstacle = nullptr; // the active hexagon while dragging

typedef std::vector<Goal *> goal_list;
goal_list goals; // the list of our obstacles
Goal *active_goal = nullptr; // the active hexagon while dragging


namespace nslo
{

// TODO: ugh
Vector2 default_robot_center = Vector2(0.50, 0.05);
double default_robot_radius = 0.05;
std::vector<Vector2> default_obstacle = 
{
    Vector2(0.40, 0.10),
    Vector2(0.20, 0.20),
    Vector2(0.30, 0.20),
    Vector2(0.50, 0.30),
    Vector2(0.60, 0.30),
    Vector2(0.55, 0.12)
};
std::vector<Vector2> default_goal = 
{
    Vector2(0.50, 0.90),
    Vector2(0.52, 0.90),
    Vector2(0.52, 0.88),
    Vector2(0.50, 0.88),
};


void init()
{
    paused = true;
    replan = true;
    // Initial objects
    robots.push_back(new Robot(default_robot_center, default_robot_radius)); 
    obstacles.push_back(new Obstacle(default_obstacle)); 
    goals.push_back(new Goal(default_goal)); 

    for (int i = 0; i < PRM_POINTS; i++)
    {
        randPoints[i] = Vector2(0.0, 0.0);
    }

    for (int i = 0; i < PRM_POINTS; i++)
    {
        for (int j = 0; j < PRM_POINTS; j++)
        {
            adjacency[i][j] = infinity;
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
}

// called on window refresh events via glut
void display()
{
    robot_list::iterator i;
    obstacle_list::iterator j;
    goal_list::iterator k;

    glClear(GL_COLOR_BUFFER_BIT); // clear color buffer

    // draw all the obstacles
    for (j = obstacles.begin(); j != obstacles.end(); ++j)
    {
        (*j)->draw();
    }

    // draw all the goals
    for (k = goals.begin(); k != goals.end(); ++k)
    {
        (*k)->draw();
    }

    // draw all the robots
    for (i = robots.begin(); i != robots.end(); ++i)
    {
        (*i)->draw();
    }

    if (SHOW_POINTS)
    {
        glColor3f(0.1, 0.5, 0.5);
        glBegin(GL_POINTS);

        for (int i = 0; i < PRM_POINTS; i++)
        {
            glVertex2f(randPoints[i].x, randPoints[i].y);
        }

        glEnd();

        for (int a = 0; a < PRM_POINTS; a++)
        {
            for (int b = 0; b < PRM_POINTS; b++)
            {
                if (adjacency[a][b] != infinity)
                {
                    glBegin(GL_LINES);
                    glVertex2f(randPoints[a].x, randPoints[a].y);
                    glVertex2f(randPoints[b].x, randPoints[b].y);
                    glEnd();
                }
            }
        }
    }

    glFlush(); // flush gl pipe
}


void step()
{
    robot_list::iterator i;

    for (i = robots.begin(); i != robots.end(); ++i)
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
            obstacle_list::iterator h;
            Vector2 tPos = (*i)->body.centroid;
            Vector2 gPos = goals.front()->body.centroid;

            for (h = obstacles.begin(); h != obstacles.end(); ++h)
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
                dx = DELTA * (*i)->orientation.x;
                dy = DELTA * (*i)->orientation.y;
                (*i)->move(dx, dy);
            }
        }
    }

    glutPostRedisplay();
}

//void idle(int id)
void idle()
{
    if (!paused)
    {
        // TODO robot proximity detection
        // TODO replan can be a (bit) vector?
        while (replan)
        {
            prm();

            bool validBounds = true;
            // TODO there's an error somewhere, maybe in when replan is set to false

            // for each triangle, find the closest point on the graph to triangle's current position
            // find closest point on graph to goal position
            // run dijkstra with those two points
            // push the resultant points onto the triangle's path
            // push goal onto the path

            robot_list::iterator tri;

            for (tri = robots.begin(); tri != robots.end(); ++tri)
            {
                if (!validBounds)
                {
                    break;
                }

                Vector2 tPos = (*tri)->body.centroid;
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
                double tDist= infinity;
                double gDist= infinity;

                for (int i = 0; i < PRM_POINTS; i++)
                {
                    bool intersect1 = false, intersect2 = false;
                    obstacle_list::iterator h;

                    for (h = obstacles.begin(); h != obstacles.end(); ++h)
                    {
                        intersect1 = (*h)->body.edge_polygon_intersect(tPos, randPoints[i]);

                        if (intersect1)
                        {
                            break;
                        }

                        intersect2 = (*h)->body.edge_polygon_intersect(gPos, randPoints[i]);

                        if (intersect2)
                        {
                            break;
                        }
                    }

                    if (intersect1 || intersect2)
                    {
                        continue;
                    }

                    double tempT = distance(tPos, randPoints[i]);
                    double tempG = distance(gPos, randPoints[i]);

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

                dijkstra(tClosest, gClosest, *tri);
            }
        }

        robot_list::iterator tri;

        for (tri = robots.begin(); tri != robots.end(); ++tri)
        {
            Vector2 q = goals.front()->body.centroid;
            (*tri)->path.push_back(q);
        }

        step();
    }
}





// called initially and on window-resizing
void reshape(int w, int h)
{
    // set the viewport to fill all of the window
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    windowWidth = w;
    windowHeight = h;

    // set top projection matrix to do 2D parallel projection, and make
    // the coordinate system of the window a unit square with origin at
    // the bottom left corner.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);
}

















// convert mouse coordinate in pixel into the window coordinate system
Vector2 mouse_to_point(int x, int y)
{
    return Vector2(double(x) / windowWidth, 1.0 - double(y) / windowHeight);
}

// called on mouse button events
void mouse_func(int button, int state, int x, int y)
{
    robot_list::iterator i;
    obstacle_list::iterator j;
    goal_list::iterator k;
    active_robot = nullptr;
    active_obstacle = nullptr;
    active_goal = nullptr;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        for (j = obstacles.begin(); j != obstacles.end(); ++j)
        {
            if ((*j)->on_down(mouse_to_point(x, y)))
            {
                active_obstacle = (*j);
                break;
            }
        }

        for (k = goals.begin(); k != goals.end(); ++k)
        {
            if ((*k)->on_down(mouse_to_point(x, y)))
            {
                active_goal = (*k);
                break;
            }
        }

        for (i = robots.begin(); i != robots.end(); ++i)
        {
            if ((*i)->on_down(mouse_to_point(x, y)))
            {
                active_robot = (*i);
                break;
            }
        }
    }
}

// called on mouse drag events
void drag(int x, int y)
{
    replan = true;
    clear_paths();

    if (active_robot != nullptr)
    {
        // drag corner of active triangle with the mouse
        active_robot->on_drag(mouse_to_point(x, y));
        glutPostRedisplay(); // force re-display
    }
    else if (active_obstacle != nullptr)
    {
        // drag corner of active hexagon with the mouse
        active_obstacle->on_drag(mouse_to_point(x, y));
        glutPostRedisplay(); // force re-display
    }
    else if (active_goal != nullptr)
    {
        // drag corner of active hexagon with the mouse
        active_goal->on_drag(mouse_to_point(x, y));
        glutPostRedisplay(); // force re-display
    }
}

// called on popup-menu invocation
void menu_func(int item)
{
    switch (item)
    {
    case 1:
        // create another triangle
        robots.push_back(new Robot(default_robot_center, default_robot_radius));
        replan = true;
        clear_paths();
        break;
    case 2:
        // create another hexagon
        obstacles.push_back(new Obstacle(default_obstacle));
        replan = true;
        clear_paths();
        break;
    case 3:
        // create another quad
        goals.push_back(new Goal(default_goal));
        replan = true;
        clear_paths();
        break;
    case 4:
        exit(0);
        break;
    default:
        break;
    }

    glutPostRedisplay(); // tell GLUT to call display()
}

void keyboard_func(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 32:
        // pause
        paused = !paused;
        break;
    case 8:
        // reset
        reset();
        break;
    default:
        break;
    }

    UNUSED(x); UNUSED(y);

    glutPostRedisplay(); // tell GLUT to call display()
}

}
