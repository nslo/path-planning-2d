#pragma once

#include "../math/vector.hpp"

namespace nslo
{

void init();

void reset();

// called on window refresh events via glut
void display();

void step();

void idle();

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

void keyboard_func(unsigned char key, int x, int y);

}
