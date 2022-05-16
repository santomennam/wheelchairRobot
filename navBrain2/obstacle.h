#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "graphics.h"

class Obstacle
{
public:
    std::vector<Vec2d> pts;
public:
    Obstacle(std::vector<Vec2d> points);
    void interpolate();
};

#endif // OBSTACLE_H
