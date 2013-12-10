#include "triangle.hpp"

namespace nslo
{

Triangle::Triangle(std::vector<Vector2> _pts)
{
    assert(_pts.size() == 3);

    // init default triangle
    num_points = _pts.size();
    pts = _pts;
    drag_point = -1;
    radius = get_radius();
    centroid = get_centroid();
}

Triangle::~Triangle() {}

void Triangle::draw()
{
    // draw the triangle
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < num_points; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }
    glEnd();

    // draw the outline
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINE_LOOP);

    for (size_t i = 0; i < num_points; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }

    glEnd();
}

bool Triangle::on_down(Vector2 p)
{
    last_mouse = p;

    // check to see if mouse is on a corner; set drag_point if so
    for (size_t i = 0; i < num_points; ++i)
    {
        if (close_enough(p, pts[i]))
        {
            drag_point = i;

            return true;
        }
    }

    // check to see if mouse is inside triangle
    if (point_in_polygon(p))
    {
        drag_point = -2;

        return true;
    }

    return false;
}

// move the drag_point corner along with the mouse
void Triangle::on_drag(Vector2 p)
{
    if (drag_point > -1)
    {
        pts[drag_point] = p;
    }

    if (drag_point == -2)
    {
        for (size_t i = 0; i < num_points; ++i)
        {
            pts[i].x += (p.x - last_mouse.x);
            pts[i].y += (p.y - last_mouse.y);
        }
    }

    last_mouse = p;
    radius = get_radius();
    centroid = get_centroid();
}

void Triangle::move(double dx, double dy)
{
    for (size_t i = 0; i < num_points; ++i)
    {
        pts[i].x += dx;
        pts[i].y += dy;
    }

    centroid = get_centroid();
}

// TODO: is this the radius or the circumference?
double Triangle::get_radius()
{
    double a = distance(pts[0], pts[1]);
    double b = distance(pts[1], pts[2]);
    double c = distance(pts[2], pts[0]);
    double radius = 2 * a * b * c /
                   std::sqrt((a + b + c) * (-a + b + c) * (a - b + c) * (a + b - c));
    return radius;
}

Vector2 Triangle::get_centroid()
{
    Vector2 s = pts[0];
    Vector2 t = pts[1];
    Vector2 u = pts[2];
    double cx = (s.x + t.x + u.x) / 3.0;
    double cy = (s.y + t.y + u.y) / 3.0;
    Vector2 centroid = Vector2(cx, cy);
    return centroid;
}

// true if this point is inside triangle described by a, b, and c
bool Triangle::point_in_polygon(Vector2 p)
{
    // Compute barycentric coordinates (u, v, w) for
    // point p with respect to triangle (a, b, c)
    double u, v, w;
    Vector2 a = pts[0];
    Vector2 b = pts[1];
    Vector2 c = pts[2];

    Vector2 v0 = Vector2(b.x - a.x, b.y - a.y);
    Vector2 v1 = Vector2(c.x - a.x, c.y - a.y);
    Vector2 v2 = Vector2(p.x - a.x, p.y - a.y);
    double d00 = dot(v0, v0);
    double d01 = dot(v0, v1);
    double d11 = dot(v1, v1);
    double d20 = dot(v2, v0);
    double d21 = dot(v2, v1);
    double denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
    bool u_ok = u >= 0.0 && u <= 1.0;
    bool v_ok = v >= 0.0 && v <= 1.0;
    bool w_ok = w >= 0.0 && w <= 1.0;

    if (u_ok && v_ok && w_ok)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// TODO: pass vectors
bool Triangle::edge_polygon_intersect(Vector2 e1, Vector2 e2)
{
    for (size_t i = 0; i < num_points; i++)
    {
        Vector2 p = pts[i];
        Vector2 q = pts[(i + 1) % num_points];
        if (edge_edge_intersect(e1.x, e1.y, e2.x, e2.y, p.x, p.y, q.x, p.y))
        {
            return true;
        }
    }

    return false;
}

}
