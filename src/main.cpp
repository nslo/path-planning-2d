#include <GL/gl.h>
#include <GL/glut.h>
#include <iostream>
#include <cassert>
#include <cmath>
#include <list>

#include "vector.hpp"

const float EPSILON = 0.01;
const float DELTA = 0.0002;
const int PRM_POINTS = 100;
const bool SHOW_POINTS = true;
const int K = 4;
const float infinity = 1000.0;

using namespace std;
using namespace _462;

//---------------- 2D point class --------------------------------//

class Point2D
{
public:
    float x,y;

    Point2D()
    {
        x = 0.0;
        y = 0.0;
    }

    Point2D(float _x, float _y) : x(_x), y(_y) {}

    double dist(Point2D a);
    bool closeEnough(Point2D other);
    bool inTriangle(Point2D* pts);
    bool inPoly(Point2D* pts, int n);
    float minDistLine(Point2D v, Point2D w);
};

// distance between this point and a
double Point2D::dist(Point2D a)
{
    return sqrt((a.x - x) * (a.x - x) + (a.y - y) * (a.y - y));
}

// true if a is close enough to this point
bool Point2D::closeEnough(Point2D a)
{
    return (dist(a) < EPSILON);
}

// true if this point is inside triangle described by a, b, and c
bool Point2D::inTriangle(Point2D* pts)
{
    // Compute barycentric coordinates (u, v, w) for
    // point p with respect to triangle (a, b, c)
    float u, v, w;
    Point2D a = pts[0];
    Point2D b = pts[1];
    Point2D c = pts[2];

    Vector2 v0 = Vector2(b.x - a.x, b.y - a.y);
    Vector2 v1 = Vector2(c.x - a.x, c.y - a.y);
    Vector2 v2 = Vector2(x - a.x, y - a.y);
    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
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

    return false;
}

// see if point is inside a polygon; cribbed from stackoverflow
bool Point2D::inPoly(Point2D* pts, int n)
{
    int i, j, c = 0;

    for (i = 0, j = n - 1; i < n; j = i++)
    {
        if (((pts[i].y > y) != (pts[j].y > y)) &&
                (x < (pts[j].x - pts[i].x) * (y - pts[i].y) /
                 (pts[j].y - pts[i].y) + pts[i].x))
        {
            c = !c;
        }
    }

    return c;
}

//------------------ interactive triangle class ------------------//

class Triangle
{
public:
    int numPoints;
    Point2D pts[3]; // the points defining the triangle
    int dragPoint; // current drag corner if any (-1 if none, -2 for move all)
    Point2D lastMouse; // last mouse position (for dragging)
    float radius; // "radius" of triangle
    Point2D centroid; // "radius" of triangle
    list<Point2D> path; // the list of points to follow to the path
    Vector2 orientation;

    Triangle();
    void draw(); // draw the triangle
    bool onDown(Point2D p); // call on mouse-down
    void onDrag(Point2D p); // call on mouse drag to move the corner
    void move(float dx, float dy);
    float getRadius(Point2D s, Point2D t, Point2D u);
    Point2D getCentroid(Point2D s, Point2D t, Point2D u);
};

Triangle::Triangle()
{
    // init default triangle
    numPoints = 3;
    pts[0] = Point2D(0.49, 0.05);
    pts[1] = Point2D(0.51, 0.05);
    pts[2] = Point2D(0.50, 0.07);
    dragPoint = -1;
    radius = getRadius(pts[0], pts[1], pts[2]);
    centroid = getCentroid(pts[0], pts[1], pts[2]);
    orientation.x = 0.0;
    orientation.y = 0.0; // no orientation to start out
}

void Triangle::draw()
{
    // draw the triangle
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_TRIANGLES);

    for (int i = 0; i < numPoints; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }

    glEnd();

    // draw the outline
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINE_LOOP);

    for (int i = 0; i < numPoints; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }

    glEnd();
}

bool Triangle::onDown(Point2D a)
{
    lastMouse = a;

    // check to see if mouse is on a corner; set dragPoint if so
    for (int i = 0; i < numPoints; ++i)
    {
        if (a.closeEnough(pts[i]))
        {
            dragPoint = i;

            return true;
        }
    }

    // check to see if mouse is inside triangle
    if (a.inTriangle(pts))
    {
        dragPoint = -2;

        return true;
    }

    return false;
}

// move the dragPoint corner along with the mouse
void Triangle::onDrag(Point2D a)
{
    if (dragPoint > -1)
    {
        pts[dragPoint] = a;
    }

    if (dragPoint == -2)
    {
        for (int i = 0; i < numPoints; ++i)
        {
            pts[i].x += (a.x - lastMouse.x);
            pts[i].y += (a.y - lastMouse.y);
        }
    }

    lastMouse = a;
    radius = getRadius(pts[0], pts[1], pts[2]);
    centroid = getCentroid(pts[0], pts[1], pts[2]);
}

void Triangle::move(float dx, float dy)
{
    for (int i = 0; i < numPoints; ++i)
    {
        pts[i].x += dx;
        pts[i].y += dy;
    }

    centroid = getCentroid(pts[0], pts[1], pts[2]);
}

float Triangle::getRadius(Point2D s, Point2D t, Point2D u)
{
    float a = s.dist(t);
    float b = t.dist(u);
    float c = u.dist(s);
    float radius = 2 * a * b * c /
                   sqrt((a + b + c) * (-a + b + c) * (a - b + c) * (a + b - c));
    return radius;
}

Point2D Triangle::getCentroid(Point2D s, Point2D t, Point2D u)
{
    float cx = (s.x + t.x + u.x) / 3.0;
    float cy = (s.y + t.y + u.y) / 3.0;
    Point2D centroid = Point2D(cx, cy);
    return centroid;
}

//------------------ interactive hexagon class ------------------//

class Hexagon
{
public:
    int numPoints;
    Point2D pts[6]; // the points defining the polygon
    int dragPoint; // current drag corner if any (-1 if none, -2 for move all)
    Point2D lastMouse; // last mouse position (for dragging)

    Hexagon();
    void draw(); // draw the polygon
    bool onDown(Point2D p); // call on mouse-down
    void onDrag(Point2D p); // call on mouse drag to move the corner
};

Hexagon::Hexagon()
{
    // init default hexagon
    numPoints = 6;
    pts[0] = Point2D(0.40, 0.10);
    pts[1] = Point2D(0.20, 0.20);
    pts[2] = Point2D(0.30, 0.20);
    pts[3] = Point2D(0.50, 0.30);
    pts[4] = Point2D(0.60, 0.30);
    pts[5] = Point2D(0.55, 0.12);
    dragPoint = -1;
}

void Hexagon::draw()
{
    // draw the hexagon
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_POLYGON);

    for (int i = 0; i < numPoints; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }

    glEnd();

    // draw the outline
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINE_LOOP);

    for (int i = 0; i < numPoints; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }

    glEnd();
}

bool Hexagon::onDown(Point2D a)
{
    lastMouse = a;

    // check to see if mouse is on a corner; set dragPoint if so
    for (int i = 0; i < numPoints; ++i)
    {
        if (a.closeEnough(pts[i]))
        {
            dragPoint = i;

            return true;
        }
    }

    // check to see if mouse is inside 
    if (a.inPoly(pts, numPoints))
    {
        dragPoint = -2;

        return true;
    }

    return false;
}

// move the dragPoint corner along with the mouse
void Hexagon::onDrag(Point2D a)
{
    if (dragPoint > -1)
    {
        pts[dragPoint] = a;
    }

    if (dragPoint == -2)
    {
        for (int i = 0; i < numPoints; ++i)
        {
            pts[i].x += (a.x - lastMouse.x);
            pts[i].y += (a.y - lastMouse.y);
        }
    }

    lastMouse = a;
}

//------------------ interactive quadrilateral class ------------------//

class Quad
{
public:
    int numPoints;
    Point2D pts[4]; // the points defining the quadrilateral
    int dragPoint; // current drag corner if any (-1 if none, -2 for move all)
    Point2D lastMouse; // last mouse position (for dragging)
    Point2D centroid;

    Quad();
    void draw(); // draw the polygon
    bool onDown(Point2D p); // call on mouse-down
    void onDrag(Point2D p); // call on mouse drag to move the corner
    Point2D getCentroid(Point2D* pts, int n);
};

Quad::Quad()
{
    // init default
    numPoints = 4;
    pts[0] = Point2D(0.49, 0.86);
    pts[1] = Point2D(0.51, 0.86);
    pts[2] = Point2D(0.51, 0.89);
    pts[3] = Point2D(0.49, 0.89);
    dragPoint = -1;
    centroid = getCentroid(pts, numPoints);
}

void Quad::draw()
{
    // draw the goal
    glColor3f(0.0, 0.8, 0.2);
    glBegin(GL_POLYGON);

    for (int i = 0; i < numPoints; ++i)
    {
        glVertex2f(pts[i].x, pts[i].y);
    }

    glEnd();
}

bool Quad::onDown(Point2D a)
{
    lastMouse = a;

    // check to see if mouse is on a corner; set dragPoint if so
    for (int i = 0; i < numPoints; ++i)
    {
        if (a.closeEnough(pts[i]))
        {
            dragPoint = i;

            return true;
        }
    }

    // check to see if mouse is inside
    if (a.inPoly(pts, numPoints))
    {
        dragPoint = -2;

        return true;
    }

    return false;
}

// move the dragPoint corner along with the mouse
void Quad::onDrag(Point2D a)
{
    if (dragPoint > -1)
    {
        pts[dragPoint] = a;
    }

    if (dragPoint == -2)
    {
        for (int i = 0; i < numPoints; ++i)
        {
            pts[i].x += (a.x - lastMouse.x);
            pts[i].y += (a.y - lastMouse.y);
        }
    }

    lastMouse = a;
    centroid = getCentroid(pts, numPoints);
}


Point2D Quad::getCentroid(Point2D* pts, int n)
{
    float cx = 0.0, cy = 0.0;

    for (int i = 0; i < n; i++)
    {
        cx += pts[i].x;
        cy += pts[i].y;
    }

    cx /= n;
    cy /= n;
    Point2D centroid = Point2D(cx, cy);

    return centroid;
}




//------------------ main program -----------------------------------//

// global variables
int windowWidth, windowHeight; // dimensions of the window in pixel
bool paused;
bool replan;
Point2D randPoints[PRM_POINTS]; 
float adjacency[PRM_POINTS][PRM_POINTS];

typedef list<Triangle *> TriList;
TriList triangles; // the list of our triangles
Triangle *activeTriangle = NULL; // the active triangle while dragging

typedef list<Hexagon *> HexList;
HexList hexagons; // the list of our hexagons
Hexagon *activeHexagon = NULL; // the active hexagon while dragging

typedef list<Quad *> QuadList;
QuadList quads; // the list of our hexagons
Quad *activeQuad = NULL; // the active hexagon while dragging


void init()
{
    paused = true;
    replan = true;
    triangles.push_back(new Triangle); // create initial triangle
    hexagons.push_back(new Hexagon); // create initial hexagon
    quads.push_back(new Quad); // create initial Quad

    for (int i = 0; i < PRM_POINTS; i++)
    {
        randPoints[i] = Point2D(0.0, 0.0);
    }

    for (int i = 0; i < PRM_POINTS; i++)
    {
        for (int j = 0; j < PRM_POINTS; j++)
        {
            adjacency[i][j] = infinity;
        }
    }

    // init some OpenGL state
    glClearColor(0.1, 0.1, 0.1, 0.0); // grey background
    glShadeModel(GL_FLAT);
}

void reset()
{
    // clear polygon lists
    triangles.clear();
    hexagons.clear();
    quads.clear();
    init();
}

// called on window refresh events via glut
void display()
{
    TriList::iterator i;
    HexList::iterator j;
    QuadList::iterator k;

    glClear(GL_COLOR_BUFFER_BIT); // clear color buffer

    // draw all the hexagons
    for (j = hexagons.begin(); j != hexagons.end(); ++j)
    {
        (*j)->draw();
    }

    // draw all the quads
    for (k = quads.begin(); k != quads.end(); ++k)
    {
        (*k)->draw();
    }

    // draw all the triangles
    for (i = triangles.begin(); i != triangles.end(); ++i)
    {
        (*i)->draw();
    }

    if (SHOW_POINTS)
    {
        glColor3f(1.0, 0.0, 0.0);
        glBegin(GL_POINTS);

        for (int i = 0; i < PRM_POINTS; i++)
        {
            glVertex2f(randPoints[i].x, randPoints[i].y);
        }

        glEnd();

        for (int a = 0; a < PRM_POINTS; a++)
        {
            for (int b = 0; b < PRM_POINTS; b++)
            {
                if (adjacency[a][b] != infinity)
                {
                    glBegin(GL_LINES);
                    glVertex2f(randPoints[a].x, randPoints[a].y);
                    glVertex2f(randPoints[b].x, randPoints[b].y);
                    glEnd();
                }
            }
        }
    }

    glFlush(); // flush gl pipe
}

// called initially and on window-resizing
void reshape(int w, int h)
{
    // set the viewport to fill all of the window
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    windowWidth = w;
    windowHeight = h;

    // set top projection matrix to do 2D parallel projection, and make
    // the coordinate system of the window a unit square with origin at
    // the bottom left corner.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);
}


static bool IsOnSegment(double xi, double yi, double xj, double yj,
        double xk, double yk) {
    return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) &&
        (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
}

static char ComputeDirection(double xi, double yi, double xj, double yj,
        double xk, double yk) {
    double a = (xk - xi) * (yj - yi);
    double b = (xj - xi) * (yk - yi);

    return a < b ? -1 : a > b ? 1 : 0;
}

/** Do line segments (x1, y1)--(x2, y2) and (x3, y3)--(x4, y4) intersect? */
bool lineSegInter(double x1, double y1, double x2, double y2,
        double x3, double y3, double x4, double y4) {
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

bool polygonInter(double x1, double y1, double x2, double y2, Point2D* pts, int n)
{
    for (int i = 0; i < n; i++)
    {
        Point2D a = pts[i];
        Point2D b = pts[(i + 1) % n];

        if (lineSegInter(x1, y1, x2, y2, a.x, a.y, b.x, b.y))
        {
            return true;
        }
    }

    return false;
}



void clearPaths()
{
    TriList::iterator tri;

    for (tri = triangles.begin(); tri != triangles.end(); ++tri)
    {
        (*tri)->path.clear();
        (*tri)->orientation.x = 0.0;
        (*tri)->orientation.y = 0.0;
    }
}



void printPath(int source, int node, int* predecessor, Triangle* tri)
{
    if (node == source)
    {
        tri->path.push_back(randPoints[source]);
        cout << (char)(node + 97) << "..";
    }
    else if (predecessor[node] == -1)
    {
        cout << "No path from “<<source<<” to "<< (char)(node + 97) << endl;
        clearPaths();
        return;
    }
    else
    {
        printPath(source, predecessor[node], predecessor, tri);
        tri->path.push_back(randPoints[node]);
        cout << (char) (node + 97) << "..";
    }

    replan = false;
}

void dijkstra(int source, int goal, Triangle* tri)
{
    // initialize; the distance between source to source is zero and all other
    // distances between source and vertices are infinity. The mark is initialized
    // to false and predecessor is initialized to -1
    int predecessor[PRM_POINTS];
    float distance[PRM_POINTS];
    bool mark[PRM_POINTS]; //keep track of visited node
    //float minDistance = infinity;
    int closestUnmarkedNode;
    int count = 0;

    for (int i = 0; i < PRM_POINTS; i++)
    {
        mark[i] = false;
        predecessor[i] = -1;
        distance[i] = infinity;
    }

    distance[source] = 0;

    while (count < PRM_POINTS)
    {
        // return the node which is nearest from the Predecessor marked node.
        // If the node is already marked as visited, then it search for another
        int minDistance = infinity;

        for(int i = 0; i < PRM_POINTS; i++)
        {
            if((!mark[i]) && (minDistance >= distance[i]))
            {
                minDistance = distance[i];
                closestUnmarkedNode = i;
            }
        }

        mark[closestUnmarkedNode] = true;

        for (int i = 0; i < PRM_POINTS; i++)
        {
            if ((!mark[i]) && (adjacency[closestUnmarkedNode][i] > 0) )
            {
                if (distance[i] > distance[closestUnmarkedNode] +
                        adjacency[closestUnmarkedNode][i])
                {
                    distance[i] = distance[closestUnmarkedNode] +
                        adjacency[closestUnmarkedNode][i];
                    predecessor[i] = closestUnmarkedNode;
                }
            }
        }

        count++;
    }

    printPath(source, goal, predecessor, tri);
    cout << "->" << distance[goal] << endl;
}

void prm()
{
    // generate 100 random points from (0.0, 0.0) to (1.0, 1.0)
    int iter = 0;
    
    while (iter < PRM_POINTS)
    {
        bool valid = true;
        float x = (float) rand() / (RAND_MAX);
        float y = (float) rand() / (RAND_MAX);
        Point2D tempPoint = Point2D(x, y);
        TriList::iterator i;
        HexList::iterator j;
        QuadList::iterator k;

        // disqualify points that:
        // a) are inside polygons
        // b) TODO are _radius_ distance from any line segment in any polygon
        // c) TODO are _radius_ distance from any vertex of any polygon

        for (j = hexagons.begin(); j != hexagons.end(); ++j)
        {
            if (tempPoint.inPoly((*j)->pts, (*j)->numPoints))
            {
                valid = false;
                break;
            }
        }

        if (!valid)
        {
            break;
        }

        for (k = quads.begin(); k != quads.end(); ++k)
        {
            if (tempPoint.inPoly((*k)->pts, (*k)->numPoints))
            {
                valid = false;
                break;
            }
        }

        if (!valid)
        {
            break;
        }

        for (i = triangles.begin(); i != triangles.end(); ++i)
        {
            if (tempPoint.inTriangle((*i)->pts))
            {
                valid = false;
                break;
            }
        }

        if (!valid)
        {
            break;
        }

        randPoints[iter] = tempPoint;
        iter++;
    }

    // turn sampled points into a graph
    // get the k-nearest-neighbors of each vertex
    // if the edge wouldn't cross an obstacle, make edge between them

    // TODO bounding boxes around polygons
    // TODO KNN (nlogn)

    // reset adjacency matrix
    for (int i = 0; i < PRM_POINTS; i++)
    {
        for (int j = 0; j < PRM_POINTS; j++)
        {
            adjacency[i][j] = infinity;
        }
    }

    for (int iter1 = 0; iter1 < PRM_POINTS; iter1++)
    {
        //int nearest[K];

        bool valid = true;
        float x1 = randPoints[iter1].x;
        float y1 = randPoints[iter1].y;

        //for (int iter2 = iter1 + 1; iter2 < PRM_POINTS; iter2++)
        for (int iter2 = iter1 + 1; iter2 < PRM_POINTS; iter2++)
        {
            TriList::iterator i;
            HexList::iterator j;
            QuadList::iterator k;
            float x2 = randPoints[iter2].x;
            float y2 = randPoints[iter2].y;

            for (j = hexagons.begin(); j != hexagons.end(); ++j)
            {
                bool intersect = polygonInter(x1, y1, x2, y2, (*j)->pts, (*j)->numPoints);

                if (intersect)
                {
                    adjacency[iter1][iter2] = infinity;
                    adjacency[iter2][iter1] = infinity;
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                break;
            }

            for (k = quads.begin(); k != quads.end(); ++k)
            {
                bool intersect = polygonInter(x1, y1, x2, y2, (*k)->pts, (*k)->numPoints);

                if (intersect)
                {
                    adjacency[iter1][iter2] = infinity;
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                break;
            }

            for (i = triangles.begin(); i != triangles.end(); ++i)
            {
                bool intersect = polygonInter(x1, y1, x2, y2, (*i)->pts, (*i)->numPoints);

                if (intersect)
                {
                    adjacency[iter1][iter2] = infinity;
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                break;
            }

            float edgeDist = randPoints[iter1].dist(randPoints[iter2]);
            adjacency[iter1][iter2] = edgeDist;
            adjacency[iter2][iter1] = edgeDist;
        }
    }

    /*
    for (int a = 0; a < PRM_POINTS; a++)
    {
        for (int b = 0; b < PRM_POINTS; b++)
        {
            cout << adjacency[a][b] << "  ";
        }

        cout << endl;
    }

    cout << endl;
    */




}

void step()
{
    TriList::iterator i;

    for (i = triangles.begin(); i != triangles.end(); ++i)
    {
        float dx, dy;

        // first check to see if the path queue is empty (reached goal)
        if (!(*i)->path.empty())
        {

            ///////////////////////
            // quick check for clear path to goal
            // TODO not sure this should be necessary if prm/dijkstra working
            // - already looking for clear path to goal before making the plan
            // - but the closest point to the goal might not actually be that close...?
            // - consider
            bool goalBlocked = false;
            HexList::iterator h;
            Point2D tPos = (*i)->centroid;
            Point2D gPos = quads.front()->centroid;

            for (h = hexagons.begin(); h != hexagons.end(); ++h)
            {
                goalBlocked |= polygonInter(tPos.x, tPos.y,
                        gPos.x, gPos.y,
                        (*h)->pts, (*h)->numPoints);
            }

            if (!goalBlocked)
            {
                (*i)->path.clear();
                (*i)->orientation.x = 0.0;
                (*i)->orientation.y = 0.0;
                Point2D q = quads.front()->centroid;
                (*i)->path.push_back(q);
            }

            ///////////////////////





            Point2D waypoint = (*i)->path.front();
            Point2D c = (*i)->centroid;

            // check to see if orientation has been set yet
            if ((*i)->orientation.x == 0.0 && (*i)->orientation.y == 0.0)
            {
                Vector2 o = Vector2(waypoint.x - c.x, waypoint.y - c.y);
                (*i)->orientation = normalize(o);
            }

            // check to see if current destination is within EPSILON of
            // next coordinate in path queue; if so, pop
            // TODO turn
            if (abs(c.y - waypoint.y) < EPSILON &&
                    (abs(c.x - waypoint.x) < EPSILON))
            {
                (*i)->path.pop_front();

                if (!(*i)->path.empty())
                {
                    waypoint = (*i)->path.front();
                    Vector2 o = Vector2(waypoint.x - c.x, waypoint.y - c.y);
                    (*i)->orientation = normalize(o);
                }
            }
            // otherwise, move toward next waypoint
            else
            {
                dx = DELTA * (*i)->orientation.x;
                dy = DELTA * (*i)->orientation.y;
                (*i)->move(dx, dy);
            }
        }
    }

    glutPostRedisplay();
}

//void idle(int id)
void idle()
{
    if (!paused)
    {
        // TODO robot proximity detection
        // TODO replan can be a (bit) vector?
        while (replan)
        {
            prm();

            bool validBounds = true;
            // TODO there's an error somewhere, maybe in when replan is set to false

            // for each triangle, find the closest point on the graph to triangle's current position
            // find closest point on graph to goal position
            // run dijkstra with those two points
            // push the resultant points onto the triangle's path
            // push goal onto the path

            TriList::iterator tri;

            for (tri = triangles.begin(); tri != triangles.end(); ++tri)
            {
                if (!validBounds)
                {
                    break;
                }

                Point2D tPos = (*tri)->centroid;
                Point2D gPos = quads.front()->centroid;

                ///////////////////////
                // quick check for clear path to goal
                /*
                bool goalBlocked = false;
                HexList::iterator h;

                for (h = hexagons.begin(); h != hexagons.end(); ++h)
                {
                    goalBlocked |= polygonInter(tPos.x, tPos.y,
                            gPos.x, gPos.y,
                            (*h)->pts, (*h)->numPoints);
                }

                if (!goalBlocked)
                {
                    // TODO fix this for muli robots
                    replan = false;
                    break;
                }
                */
                ///////////////////////


                // check to see if closest nodes to start and goal are valid
                int tClosest = -1;
                int gClosest = -1;
                float tDist= infinity;
                float gDist= infinity;

                for (int i = 0; i < PRM_POINTS; i++)
                {
                    bool intersect1 = false, intersect2 = false;
                    HexList::iterator h;

                    for (h = hexagons.begin(); h != hexagons.end(); ++h)
                    {
                        intersect1 = polygonInter(tPos.x, tPos.y,
                                randPoints[i].x, randPoints[i].y,
                                (*h)->pts, (*h)->numPoints);

                        if (intersect1)
                        {
                            break;
                        }

                        intersect2 = polygonInter(gPos.x, gPos.y,
                                randPoints[i].x, randPoints[i].y,
                                (*h)->pts, (*h)->numPoints);

                        if (intersect2)
                        {
                            break;
                        }
                    }

                    if (intersect1 || intersect2)
                    {
                        continue;
                    }

                    float tempT = tPos.dist(randPoints[i]);
                    float tempG = gPos.dist(randPoints[i]);

                    if (tempT < tDist)
                    {
                        tClosest = i;
                        tDist = tempT;
                    }

                    if (tempG < gDist)
                    {
                        gClosest = i;
                        gDist = tempG;
                    }
                }

                if (tClosest == -1 || gClosest == -1)
                {
                    validBounds = false;
                    break;
                }

                dijkstra(tClosest, gClosest, *tri);
            }
        }

        TriList::iterator tri;

        for (tri = triangles.begin(); tri != triangles.end(); ++tri)
        {
            Point2D q = quads.front()->centroid;
            (*tri)->path.push_back(q);
        }

        step();
    }
}

// convert mouse coordinate in pixel into the window coordinate system
Point2D mouseToPoint(int x, int y)
{
    return Point2D(float(x) / windowWidth, 1.0 - float(y) / windowHeight);
}

// called on mouse button events
void mouse(int button, int state, int x, int y)
{
    TriList::iterator i;
    HexList::iterator j;
    QuadList::iterator k;
    activeTriangle = NULL;
    activeHexagon = NULL;
    activeQuad = NULL;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        for (j = hexagons.begin(); j != hexagons.end(); ++j)
        {
            if ((*j)->onDown(mouseToPoint(x, y)))
            {
                activeHexagon = (*j);
                break;
            }
        }

        for (k = quads.begin(); k != quads.end(); ++k)
        {
            if ((*k)->onDown(mouseToPoint(x, y)))
            {
                activeQuad = (*k);
                break;
            }
        }

        for (i = triangles.begin(); i != triangles.end(); ++i)
        {
            if ((*i)->onDown(mouseToPoint(x, y)))
            {
                activeTriangle = (*i);
                break;
            }
        }
    }
}

// called on mouse drag events
void drag(int x, int y)
{
    replan = true;
    clearPaths();

    if (activeTriangle != NULL)
    {
        // drag corner of active triangle with the mouse
        activeTriangle->onDrag(mouseToPoint(x, y));
        glutPostRedisplay(); // force re-display
    }
    else if (activeHexagon != NULL)
    {
        // drag corner of active hexagon with the mouse
        activeHexagon->onDrag(mouseToPoint(x, y));
        glutPostRedisplay(); // force re-display
    }
    else if (activeQuad != NULL)
    {
        // drag corner of active hexagon with the mouse
        activeQuad->onDrag(mouseToPoint(x, y));
        glutPostRedisplay(); // force re-display
    }
}

// called on popup-menu invocation
void menuFunc(int item)
{
    switch (item)
    {
    case 1:
        // create another triangle
        triangles.push_back(new Triangle);
        replan = true;
        clearPaths();
        break;
    case 2:
        // create another hexagon
        hexagons.push_back(new Hexagon);
        replan = true;
        clearPaths();
        break;
    case 3:
        // create another quad
        quads.push_back(new Quad);
        replan = true;
        clearPaths();
        break;
    case 4:
        exit(0);
        break;
    default:
        break;
    }

    glutPostRedisplay(); // tell GLUT to call display()
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 32:
        // pause
        paused = !paused;
        break;
    case 8:
        // reset
        reset();
        break;
    default:
        break;
    }

    x = y; y = x; // suppress compiler warnings (x and y are mouse coords at time of keypress)

    glutPostRedisplay(); // tell GLUT to call display()
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv); // init GLUT library
    glutInitDisplayMode(GLUT_SINGLE|GLUT_RGB); // true color and single buffer
    glutInitWindowSize(1600, 1024); // set initial window size
    glutCreateWindow("Float"); // open window with a title

    init();

    // register callback functions for redraw, mouse events etc.
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(drag);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);

    // animation and stuff goes here
    glutIdleFunc(idle);

    // create right button popup menu
    glutCreateMenu(menuFunc);
    glutAddMenuEntry("New Robot", 1);
    glutAddMenuEntry("New Obstacle", 2);
    glutAddMenuEntry("New Goal", 3);
    glutAddMenuEntry("Quit", 4);
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    glutMainLoop(); // enter main loop

    return 0;
}
