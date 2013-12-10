#include "draw.hpp"
#include "../map/map.hpp"
#include "../map/prm.hpp"
#include "../map/dijkstra.hpp"
#include <vector>

#define UNUSED(expr) do { (void)(expr); } while (0)

namespace nslo
{

// called on window refresh events via glut
void display()
{
    glClear(GL_COLOR_BUFFER_BIT); // clear color buffer

    // draw all the obstacles
    for (auto j = obstacles.begin(); j != obstacles.end(); ++j)
    {
        (*j)->draw();
    }

    // draw all the goals
    for (auto k = goals.begin(); k != goals.end(); ++k)
    {
        (*k)->draw();
    }

    // draw all the robots
    for (auto i = robots.begin(); i != robots.end(); ++i)
    {
        (*i)->draw();
    }

    if (SHOW_POINTS)
    {
        glColor3f(0.1, 0.5, 0.5);
        glBegin(GL_POINTS);

        for (size_t i = 0; i < PRM_POINTS; i++)
        {
            glVertex2f(rand_points[i].x, rand_points[i].y);
        }

        glEnd();

        for (size_t a = 0; a < PRM_POINTS; a++)
        {
            for (size_t b = 0; b < PRM_POINTS; b++)
            {
                if (adjacency[a][b] != infinity)
                {
                    glBegin(GL_LINES);
                    glVertex2f(rand_points[a].x, rand_points[a].y);
                    glVertex2f(rand_points[b].x, rand_points[b].y);
                    glEnd();
                }
            }
        }
    }

    glFlush(); // flush gl pipe
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
    active_robot = nullptr;
    active_obstacle = nullptr;
    active_goal = nullptr;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        for (auto j = obstacles.begin(); j != obstacles.end(); ++j)
        {
            if ((*j)->on_down(mouse_to_point(x, y)))
            {
                active_obstacle = (*j);
                break;
            }
        }

        for (auto k = goals.begin(); k != goals.end(); ++k)
        {
            if ((*k)->on_down(mouse_to_point(x, y)))
            {
                active_goal = (*k);
                break;
            }
        }

        for (auto i = robots.begin(); i != robots.end(); ++i)
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
    clear_paths(robots);

    if (active_robot != nullptr)
    {
        // drag corner of active thing
        active_robot->on_drag(mouse_to_point(x, y));
        glutPostRedisplay(); // force re-display
    }
    else if (active_obstacle != nullptr)
    {
        // drag corner of active thing
        active_obstacle->on_drag(mouse_to_point(x, y));
        glutPostRedisplay(); // force re-display
    }
    else if (active_goal != nullptr)
    {
        // drag corner of active thing
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
    {
        // create another robot
        Vector2 center;
        double radius;
        get_new_robot(center, radius);
        robots.push_back(new Robot(center, radius));
        replan = true;
        clear_paths(robots);
        break;
    }
    case 2:
    {
        // create another obstacle
        std::vector<Vector2> obstacle_pts;
        get_new_obstacle(obstacle_pts);
        obstacles.push_back(new Obstacle(obstacle_pts));
        replan = true;
        clear_paths(robots);
        break;
    }
    case 3:
    {
        // create another goal
        std::vector<Vector2> goal_pts;
        get_new_goal(goal_pts);
        goals.push_back(new Goal(goal_pts));
        replan = true;
        clear_paths(robots);
        break;
    }
    case 4:
    {
        exit(0);
        break;
    }
    default:
    {
        break;
    }
    }

    glutPostRedisplay(); // tell GLUT to call display()
}

void keyboard_func(unsigned char key, int x, int y)
{
    switch (key)
    {
    // Spacebar
    case 32:
        // pause
        paused = !paused;
        break;
    // R?
        /*
    case 8:
        // reset
        reset();
        break;
        */
    default:
        break;
    }

    UNUSED(x); UNUSED(y);

    glutPostRedisplay(); // tell GLUT to call display()
}

}
