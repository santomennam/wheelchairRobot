#ifndef VIEWPORT_H
#define VIEWPORT_H
#include "graphics.h"

class Viewport
{
public:
    void reset(Vec2d robotPos);
    Viewport();
    Vec2d worldToScreen(Vec2d);
    std::vector<Vec2d> worldToScreen(std::vector<Vec2d> points);
    Vec2d screenToWorld(Vec2d);
    void pan(Vec2d offset);
    void zoom(Vec2d zoomCenter, double scale);
public:
    double scale = 1.0; //world to screen
    Vec2d panOffset = {0,0}; //offset in screen coordinates
};

#endif // VIEWPORT_H
