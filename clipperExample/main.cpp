#include <iostream>
#include "graphics.h"
#include <iomanip>
#include "clipper.h"

using namespace std;
using namespace mssm;
using namespace ClipperLib;

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

// Note:  Mac Users who are using Qt Creator...
//        if the program compiles and runs but a window doesn't open up
//        try un-checking "run in terminal"

// Here are some of the basic drawing commands:

//void   line(Vec2d p1, Vec2d p2, Color c = WHITE);
//void   ellipse(Vec2d center, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
//void   arc(Vec2d center, double w, double h, double a, double alen, Color c = WHITE);
//void   chord(Vec2d center, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
//void   pie(Vec2d center, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
//void   rect(Vec2d corner, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
//void   polygon(std::vector<Vec2d> pts, Color border, Color fill = TRANSPARENT);
//void   polyline(std::vector<Vec2d> pts, Color color);
//void   text(Vec2d pos, double size, const std::string& str, Color textColor = WHITE, HAlign hAlign = HAlign::left, VAlign vAlign = VAlign::baseline);

Path makePath(const vector<Vec2d>& pts)
{
    Path path;
    for (const auto& p : pts)
    {
        path.push_back(IntPoint(p.x, p.y));
    }
    return path;
}

Paths makePaths(const vector<vector<Vec2d>>& polys)
{
    Paths paths;
    for (const auto& p : polys)
    {
        paths.push_back(makePath(p));
    }
    return paths;
}

vector<Vec2d> makePoly(const Path& path)
{
    vector<Vec2d> pts;
    for (const auto& p : path)
    {
        pts.push_back({p.X, p.Y});
    }
    return pts;
}

vector<vector<Vec2d>> makePolys(const Paths& paths)
{
    vector<vector<Vec2d>> polys;
    for (const auto& p : paths)
    {
        polys.push_back(makePoly(p));
    }
    return polys;
}

vector<vector<Vec2d>> clip(vector<Vec2d> poly1, vector<Vec2d> poly2, ClipType ct)
{
    Clipper c;

    c.AddPath(makePath(poly1), PolyType::ptSubject, true);
    c.AddPath(makePath(poly2), PolyType::ptClip, true);

    Paths pathsOut;

    c.Execute(ct, pathsOut);

    return makePolys(pathsOut);
}


vector<vector<Vec2d>> clip(vector<vector<Vec2d>> polys, vector<Vec2d> poly, ClipType ct)
{
    Clipper c;

    c.AddPaths(makePaths(polys), PolyType::ptSubject, true);
    c.AddPath(makePath(poly), PolyType::ptClip, true);

    Paths pathsOut;

    c.Execute(ct, pathsOut);

    return makePolys(pathsOut);
}

vector<Vec2d> makeCircle(double radius, int numPoints, Vec2d pos)
{
    Vec2d  p{radius, 0};
    double delta = M_PI*2.0/numPoints;
    vector<Vec2d> pts;
    double angle = 0.0;
    for (int i = 0; i < numPoints; i++) {
        pts.push_back(pos + p.rotated(angle));
        angle += delta;
    }
    return pts;
}

vector<vector<Vec2d>> simplify(vector<vector<Vec2d>> polys)
{
    auto paths = makePaths(polys);
    Paths pathsOut;
    CleanPolygons(paths, pathsOut, 2);
    return makePolys(pathsOut);
}

int main()
{
    Graphics g("My Program", 1024, 768);

    vector<Vec2d> poly1{{20,20}, {200,50}, {250,200}, {40, 260}};
    vector<Vec2d> poly2{{80,20}, {220,50}, {290,200}, {70, 260}};

    vector<vector<Vec2d>> polys;


    while (g.draw()) {

        g.polygon(poly1, GREY);
        g.polygon(poly2, GREY);

        for (const auto& p : polys) {
            g.points(p, GREEN);
            g.polygon(p, YELLOW);
        }

        if (g.isKeyPressed(Key::ESC)) {
            break;
        }

        if (g.isKeyPressed(' ')) {
            g.cout << "<SPACE>" << endl;
        }

        if (g.onKeyPress(' ')) {
            g.cerr << "Space Press" << endl;
        }
        if (g.onKeyRelease(' ')) {
            g.cerr << "Space Release" << endl;
        }

        if (g.onMousePress(1)) {
            polys = clip(polys, makeCircle(30, 6, g.mousePos()), ClipType::ctUnion);
        }

        switch (g.getKeyPressed()) {
        case Key::None:
            break;
        case Key::Down:
            g.cerr << "I'm down with that" << endl;
            break;
        default:
            g.cerr << "Key Pressed: " << g.getKeyPressed() << endl;
            break;
        }

        g.polygon(makeCircle(30, 12, g.mousePos()), GREEN);

        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
                switch (e.arg) {
                case 'U':
                    polys = clip(poly1, poly2, ClipType::ctUnion);
                    break;
                case 'D':
                    polys = clip(poly1, poly2, ClipType::ctDifference);
                    break;
                case 'I':
                    polys = clip(poly1, poly2, ClipType::ctIntersection);
                    break;
                case 'X':
                    polys = clip(poly1, poly2, ClipType::ctXor);
                    break;
                case 'S':
                    polys = simplify(polys);
                    break;
                case Key::ENTER:
                    break;
                }

                break;
            case EvtType::KeyRelease:
                break;
            case EvtType::MouseMove:
                polys = clip(polys, makeCircle(30, 12, g.mousePos()), ClipType::ctUnion);
                break;
            case EvtType::MousePress:

<<<<<<< HEAD
                polys = clip(polys, makeCircle(30, 12, g.mousePos()), ClipType::ctUnion);
=======
>>>>>>> ea74b689165e841aa43ef21addf9fe31333e14bb


                break;
            case EvtType::MouseRelease:
                break;
            default:
                break;
            }
        }
    }

    return 0;
}


