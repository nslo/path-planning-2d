#pragma once

#include <GL/gl.h>
#include <GL/glut.h>
#include "math/vector.hpp"

const double PI = 3.1415926;
const double EPSILON = 0.01;

namespace nslo 
{

// Some things can be drawn, some things can be drawn and moved
class Drawable
{
    // Constructor and destructor
    Drawable();
    virtual ~Drawable();
    
    // Draw something
    virtual void draw() = 0;
};

class Moveable : public Drawable
{
public:
    // Constructor and Destructor
    Moveable();
    virtual ~Moveable();

    // Member variables
    int drag_point; // current drag corner if any (-1 if none, -2 for move all)
    Vector2 last_mouse; // last mouse position (for dragging)

    // Drawing functions
    virtual bool on_down(Vector2 p) = 0; // call on mouse-down
    virtual void on_drag(Vector2 p) = 0; // call on mouse drag to move the corner
    virtual void move(double dx, double dy) = 0;
};

}
