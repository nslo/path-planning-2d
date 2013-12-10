#pragma once

#include "draw/drawable.hpp"
#include "geometry/circle.hpp"
#include "geometry/polygon.hpp"
#include <list>

namespace nslo
{

class Robot : public Drawable
{
public:
    // Constructor and destructor
    Robot(const Vector2& _center, const double& _radius);
    ~Robot();

    // Member variables
    Circle body; // polygon representation (no meshes for now)
    std::list<Vector2> path; // the list of waypoints to follow to the goal
    Vector2 goal; // position of this robot's goal
    Vector2 orientation;
    
    // Drawing functions
    virtual void draw(); // draw the triangle
    virtual bool on_down(Vector2 p); // call on mouse-down
    virtual void on_drag(Vector2 p); // call on mouse drag to move the corner
    virtual void move(double dx, double dy);
};

class Obstacle : public Drawable 
{
public:
    // Constructor and destructor
    Obstacle(const std::vector<Vector2>& _pts);
    ~Obstacle();

    // Member variables
    Polygon body;
    
    // Drawing functions
    virtual void draw(); // draw the triangle
    virtual bool on_down(Vector2 p); // call on mouse-down
    virtual void on_drag(Vector2 p); // call on mouse drag to move the corner
    virtual void move(double dx, double dy);
};

class Goal : public Drawable
{
public:
    // Constructor and destructor
    Goal(const std::vector<Vector2>& _pts);
    ~Goal();

    // Member variables
    Polygon body;
    
    // Drawing functions
    virtual void draw(); // draw the triangle
    virtual bool on_down(Vector2 p); // call on mouse-down
    virtual void on_drag(Vector2 p); // call on mouse drag to move the corner
    virtual void move(double dx, double dy);
};

}
