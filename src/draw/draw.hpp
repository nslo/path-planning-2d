#pragma once

#include "../math/vector.hpp"
#include "../map/map.hpp"

namespace nslo
{

// global variables
std::vector<Robot *> robots; // the list of our robots
Robot *active_robot = nullptr; // the active triangle while dragging
std::vector<Obstacle *> obstacles; // the list of our obstacles
Obstacle *active_obstacle = nullptr; // the active hexagon while dragging
std::vector<Goal *> goals; // the list of our obstacles
Goal *active_goal = nullptr; // the active hexagon while dragging
std::vector<Vector2> rand_points;
bool paused;
bool replan;
int windowWidth, windowHeight; // dimensions of the window in pixel
const double DELTA = 0.0002;
const bool SHOW_POINTS = true;

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
