#include <iostream>
#include <limits>
#include <vector>
#include "map/map.hpp"
#include "draw/draw.hpp"

#define UNUSED(expr) do { (void)(expr); } while (0)

using namespace nslo;

int main(int argc, char *argv[])
{
    glutInit(&argc, argv); // init GLUT library
    glutInitDisplayMode(GLUT_SINGLE|GLUT_RGB); // true color and single buffer
    glutInitWindowSize(1600, 1024); // set initial window size
    glutCreateWindow("Float"); // open window with a title

    init();

    // register callback functions for redraw, mouse events etc.
    glutDisplayFunc(display);
    glutMouseFunc(mouse_func);
    glutMotionFunc(drag);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard_func);

    // animation and stuff goes here
    glutIdleFunc(idle);

    // create right button popup menu
    glutCreateMenu(menu_func);
    glutAddMenuEntry("New Robot", 1);
    glutAddMenuEntry("New Obstacle", 2);
    glutAddMenuEntry("New Goal", 3);
    glutAddMenuEntry("Quit", 4);
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    glutMainLoop(); // enter main loop

    return 0;
}
