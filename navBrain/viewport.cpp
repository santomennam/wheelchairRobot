#include "viewport.h"

void Viewport::reset(Vec2d robotPos)
{

}

Viewport::Viewport()
{

}

Vec2d Viewport::worldToScreen(Vec2d vec)
{
    //scale: pixels per inch
    //offset: pixels

    return(vec*scale) + panOffset;
}

std::vector<Vec2d> Viewport::worldToScreen(std::vector<Vec2d> points)
{
    for(auto& p : points)
    {
        p = worldToScreen(p);
    }
    return points;
}

Vec2d Viewport::screenToWorld(Vec2d vec)
{
    vec = (vec-panOffset)*(1/scale);
    return vec;
}

void Viewport::pan(Vec2d offset) //offset must be given in pixels
{
    panOffset = panOffset+offset;
}

void Viewport::zoom(Vec2d zoomCenter, double scale) //scale is a multiplier
{
    this->scale *= scale;
    panOffset = (scale*(panOffset-zoomCenter))+zoomCenter;
}
