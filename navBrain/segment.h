#ifndef SEGMENT_H
#define SEGMENT_H
#include "vec2d.h"
#include <vector>
#include <algorithm>
#include <graphics.h>

#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wnarrowing"

class segment
{
public:
    Vec2d p1;
    Vec2d p2;
    bool open;
    mssm::Color C = mssm::WHITE;
public:
    void save(std::ostream& strm);
    void load(std::istream& strm);
    segment();

    double magnitude()  {return (p1-p2).magnitude();}
    void magChecker(double width);
    segment(Vec2d point1, Vec2d point2, double botWidth);
    double distanceToPoint(Vec2d point, Vec2d& closest);
    double distanceToPoint(Vec2d point, Vec2d& closest, double& t);
    Vec2d midpoint() const{return (p1+p2)*0.5;}
};

double crossSeg(segment a, segment b);
bool operator==(segment a, segment b);


#endif // SEGMENT_H
