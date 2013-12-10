#include "polygon.hpp"

namespace nslo
{

Polygon::Polygon(const std::vector<Vector2>& _pts)
{
    assert(_pts.size() > 2);

    // init default polygon
    num_points = _pts.size();
    pts = _pts;
    drag_point = -1;
}

Polygon::~Polygon()
{
}

void Polygon::draw()
{
    // draw the polygon
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_POLYGON);

    for (size_t i = 0; i < num_points; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }

    glEnd();

    // draw the outline
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINE_LOOP);

    for (size_t i = 0; i < num_points; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }

    glEnd();
}

bool Polygon::on_down(Vector2 p)
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

    // check to see if mouse is inside 
    if (point_in_polygon(p))
    {
        drag_point = -2;

        return true;
    }

    return false;
}

// move the drag_point corner along with the mouse
void Polygon::on_drag(Vector2 p)
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
}

void Polygon::move(double dx, double dy)
{
    for (size_t i = 0; i < num_points; ++i)
    {
        pts[i].x += dx;
        pts[i].y += dy;
    }

    centroid = get_centroid();
}

Vector2 Polygon::get_centroid()
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (size_t i = 0; i < pts.size(); ++i)
    {
        sum_x += pts[i].x;
        sum_y += pts[i].y;
    }
    centroid = Vector2(sum_x / pts.size(), sum_y / pts.size());
    
    return centroid;
}

// see if point is inside a polygon; cribbed from stackoverflow
bool Polygon::point_in_polygon(Vector2 p)
{
    size_t i, j, c = 0;
    for (i = 0, j = num_points - 1; i < num_points; j = i++)
    {
        if (((pts[i].y > p.y) != (pts[j].y > p.y)) &&
                (p.x < (pts[j].x - pts[i].x) * (p.y - pts[i].y) /
                 (pts[j].y - pts[i].y) + pts[i].x))
        {
            c = !c;
        }
    }

    return c;
}

// TODO: pass vectors
bool Polygon::edge_polygon_intersect(Vector2 e1, Vector2 e2)
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
