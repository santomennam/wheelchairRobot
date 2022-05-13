#include "polyops.h"
using namespace std;
using namespace ClipperLib;
PolyOps::PolyOps()
{}

Path PolyOps::makePath(const std::vector<Vec2d> &pts)
{
    Path path;
    for (const auto& p : pts)
    {
        path.push_back(IntPoint(p.x, p.y));
    }
    return path;
}

ClipperLib::Paths PolyOps::makePaths(const std::vector<std::vector<Vec2d> > &polys)
{
    Paths paths;
    for (const auto& p : polys)
    {
        paths.push_back(makePath(p));
    }
    return paths;
}

std::vector<Vec2d> PolyOps::makePoly(const ClipperLib::Path &path)
{
    vector<Vec2d> pts;
    for (const auto& p : path)
    {
        pts.push_back(toVec2d(p));
    }
    return pts;
}

std::vector<std::vector<Vec2d> > PolyOps::makePolys(const ClipperLib::Paths &paths)
{
    vector<vector<Vec2d>> polys;
    for (const auto& p : paths)
    {
        polys.push_back(makePoly(p));
    }
    return polys;
}

std::vector<std::vector<Vec2d> > PolyOps::clip(std::vector<Vec2d> poly1, std::vector<Vec2d> poly2, ClipperLib::ClipType ct)
{
    Clipper c;

    c.AddPath(makePath(poly1), PolyType::ptSubject, true);
    c.AddPath(makePath(poly2), PolyType::ptClip, true);

    Paths pathsOut;

    c.Execute(ct, pathsOut);

    return makePolys(pathsOut);
}

std::vector<std::vector<Vec2d> > PolyOps::clip(std::vector<std::vector<Vec2d> > polys, std::vector<Vec2d> poly, ClipperLib::ClipType ct)
{
    Clipper c;

    c.AddPaths(makePaths(polys), PolyType::ptSubject, true);
    c.AddPath(makePath(poly), PolyType::ptClip, true);

    Paths pathsOut;

    c.Execute(ct, pathsOut);

    return makePolys(pathsOut);
}

std::vector<Vec2d> PolyOps::makeCircle(double radius, int numPoints, Vec2d pos)
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

std::vector<std::vector<Vec2d> > PolyOps::simplify(std::vector<std::vector<Vec2d> > polys)
{
    auto paths = makePaths(polys);
    Paths pathsOut;
    CleanPolygons(paths, pathsOut, 2);
    return makePolys(pathsOut);
}
