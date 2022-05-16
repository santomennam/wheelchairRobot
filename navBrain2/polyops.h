#ifndef POLYOPS_H
#define POLYOPS_H

#include "clipper.h"
#include "vec2d.h"
#include <iomanip>

class PolyOps
{
    bool   useScaling{false};
    double offset;
    double scale;
public:
    std::vector<std::vector<Vec2d>> polys;

public:
    PolyOps();

    static constexpr ClipperLib::IntPoint toIP(const Vec2d& p) { return ClipperLib::IntPoint(p.x*100, p.y*100); }
    static constexpr Vec2d toVec2d(const ClipperLib::IntPoint& p) { return Vec2d(p.X/100.0, p.Y/100.0); }

//    ClipperLib::IntPoint toIPs(const Vec2d& p);
//    Vec2d toVec2ds(const ClipperLib::IntPoint& p);

    ClipperLib::Path makePath(const std::vector<Vec2d>& pts);
    ClipperLib::Paths makePaths(const std::vector<std::vector<Vec2d>>& polys);
    std::vector<Vec2d> makePoly(const ClipperLib::Path& path);
    std::vector<std::vector<Vec2d>> makePolys(const ClipperLib::Paths& paths);
    std::vector<std::vector<Vec2d>> clip(std::vector<Vec2d> poly1, std::vector<Vec2d> poly2, ClipperLib::ClipType ct);
    std::vector<std::vector<Vec2d>> clip(std::vector<std::vector<Vec2d>> polys, std::vector<Vec2d> poly, ClipperLib::ClipType ct);
    std::vector<Vec2d> makeCircle(double radius, int numPoints, Vec2d pos);
    std::vector<std::vector<Vec2d>> simplify(std::vector<std::vector<Vec2d>> polys);


};

#endif // POLYOPS_H
