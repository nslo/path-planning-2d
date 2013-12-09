#include "geometry.hpp"

const double EPSILON = 0.01;

namespace nslo {

// true if a is close enough to this point
bool close_enough(Vector2 a, Vector2 b)
{
    return (distance(a, b) < EPSILON);
}

// true if this point is inside triangle described by a, b, and c
bool point_in_triangle(Vector2 p, Vector2* pts)
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

// see if point is inside a polygon; cribbed from stackoverflow
bool point_in_polygon(Vector2 p, Vector2* pts, int n)
{
    int i, j, c = 0;

    for (i = 0, j = n - 1; i < n; j = i++)
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

static bool IsOnSegment(double xi, double yi, double xj, double yj,
        double xk, double yk)
{
    return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) &&
        (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
}

static char ComputeDirection(double xi, double yi, double xj, double yj,
        double xk, double yk)
{
    double a = (xk - xi) * (yj - yi);
    double b = (xj - xi) * (yk - yi);

    return a < b ? -1 : a > b ? 1 : 0;
}

/** Do line segments (x1, y1)--(x2, y2) and (x3, y3)--(x4, y4) intersect? */
bool edge_edge_intersection(double x1, double y1, double x2, double y2,
        double x3, double y3, double x4, double y4)
{
    char d1 = ComputeDirection(x3, y3, x4, y4, x1, y1);
    char d2 = ComputeDirection(x3, y3, x4, y4, x2, y2);
    char d3 = ComputeDirection(x1, y1, x2, y2, x3, y3);
    char d4 = ComputeDirection(x1, y1, x2, y2, x4, y4);

    return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) ||
        (d1 == 0 && IsOnSegment(x3, y3, x4, y4, x1, y1)) ||
        (d2 == 0 && IsOnSegment(x3, y3, x4, y4, x2, y2)) ||
        (d3 == 0 && IsOnSegment(x1, y1, x2, y2, x3, y3)) ||
        (d4 == 0 && IsOnSegment(x1, y1, x2, y2, x4, y4));
}


// TODO: pass vectors
bool edge_polygon_intersection(Vector2 e1, Vector2 e2, Vector2* pts, int n)
{
    for (int i = 0; i < n; i++)
    {
        Vector2 p = pts[i];
        Vector2 q = pts[(i + 1) % n];

        if (edge_edge_intersection(e1.x, e1.y, e2.x, e2.y, p.x, p.y, q.x, p.y))
        {
            return true;
        }
    }

    return false;
}





}
