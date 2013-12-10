#include "map.hpp"

namespace nslo
{

void get_new_robot(Vector2& center, double& radius)    
{
    Vector2 default_robot_center = Vector2(0.50, 0.05);
    double default_robot_radius = 0.05;
    center = default_robot_center;
    radius = default_robot_radius;
}


void get_new_obstacle(std::vector<Vector2>& obstacle_pts)
{
    std::vector<Vector2> default_obstacle = 
    {
        Vector2(0.40, 0.10),
        Vector2(0.20, 0.20),
        Vector2(0.30, 0.20),
        Vector2(0.50, 0.30),
        Vector2(0.60, 0.30),
        Vector2(0.55, 0.12)
    };
    obstacle_pts = default_obstacle;
}

void get_new_goal(std::vector<Vector2>& goal_pts)
{
    std::vector<Vector2> default_goal = 
    {
        Vector2(0.50, 0.90),
        Vector2(0.52, 0.90),
        Vector2(0.52, 0.88),
        Vector2(0.50, 0.88),
    };
    goal_pts = default_goal;
}

Robot::Robot(const Vector2& _center, const double& _radius) 
    : body(_center, _radius)
{
    orientation = Vector2(0.0, 0.0);
}

Robot::~Robot() {}

void Robot::draw()
{
    return body.draw();
}

bool Robot::on_down(Vector2 p)
{
    return body.on_down(p);
}

void Robot::on_drag(Vector2 p)
{
    return body.on_drag(p);
}

void Robot::move(double dx, double dy)
{
    return body.move(dx, dy);
}

Obstacle::Obstacle(const std::vector<Vector2>& _pts)
: body(_pts) {}

Obstacle::~Obstacle() {}

void Obstacle::draw()
{
    return body.draw();
}

bool Obstacle::on_down(Vector2 p)
{
    return body.on_down(p);
}

void Obstacle::on_drag(Vector2 p)
{
    return body.on_drag(p);
}

void Obstacle::move(double dx, double dy)
{
    return body.move(dx, dy);
}


Goal::Goal(const std::vector<Vector2>& _pts)
    : body(_pts) {}

Goal::~Goal() {}

void Goal::draw()
{
    return body.draw();
}

bool Goal::on_down(Vector2 p)
{
    return body.on_down(p);
}

void Goal::on_drag(Vector2 p)
{
    return body.on_drag(p);
}

void Goal::move(double dx, double dy)
{
    return body.move(dx, dy);
}

}
