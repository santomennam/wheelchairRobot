#ifndef POLYOPS_H
#define POLYOPS_H

#include "clipper.h"
#include "vec2d.h"

class PolyOps
{
    bool   useScaling{false};
    double offset;
    double scale;
public:
    PolyOps();

    static constexpr ClipperLib::IntPoint toIP(const Vec2d& p) { return ClipperLib::IntPoint(p.x, p.y); }
    static constexpr Vec2d toVec2d(const ClipperLib::IntPoint& p) { return Vec2d{p.X, p.Y}; }

    ClipperLib::IntPoint toIPs(const Vec2d& p);
    Vec2d toVec2ds(const ClipperLib::IntPoint& p);
};

#endif // POLYOPS_H
