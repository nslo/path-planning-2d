#pragma once

#include "../math/vector.hpp"
#include "../map/map.hpp"
#include "../map/dijkstra.hpp"

namespace nslo
{

// global variables
extern std::vector<Robot *> robots; // the list of our robots
extern Robot *active_robot; // the active triangle while dragging
extern std::vector<Obstacle *> obstacles; // the list of our obstacles
extern Obstacle *active_obstacle; // the active hexagon while dragging
extern std::vector<Goal *> goals; // the list of our obstacles
extern Goal *active_goal; // the active hexagon while dragging
extern std::vector<Vector2> rand_points;
extern bool paused;
extern bool replan;
extern int windowWidth, windowHeight; // dimensions of the window in pixel

extern bool SHOW_POINTS;
extern size_t PRM_POINTS;
extern double infinity;
extern std::vector<std::vector<double>> adjacency;

// called on window refresh events via glut
void display();

// called initially and on window-resizing
void reshape(int w, int h);

// convert mouse coordinate in pixel into the window coordinate system
Vector2 mouse_to_point(int x, int y);

// called on mouse button events
void mouse_func(int button, int state, int x, int y);

// called on mouse drag events
void drag(int x, int y);

// called on popup-menu invocation
void menu_func(int item);

// keyboard callback
void keyboard_func(unsigned char key, int x, int y);

}
