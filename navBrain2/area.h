#ifndef AREA_H
#define AREA_H
#include "segment.h"

class Area
{
public:
    std::vector<segment> boundaries;
public:
    Area(std::vector<segment>segments);
};

#endif // AREA_H
