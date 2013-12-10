#include "map.hpp"

namespace nslo
{

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
