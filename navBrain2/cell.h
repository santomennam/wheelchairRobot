#ifndef CELL_H
#define CELL_H
#include "graphics.h"

class Grid;
class cell
{
public:
    bool inQ;
    Grid* parentGrid;
    Vec2d centroid; //in world coords
    bool visited = false;
    bool blocked = false;
    Vec2i32 indices;
    mssm::Color color = mssm::BLACK;
public:
    void draw(mssm::Graphics& g);
    cell(Grid* grid,Vec2d centroid,Vec2d indices);
    double distanceToPoint(Vec2d point, Vec2d& closest);
    bool pointInCell(Vec2d point);
};

#endif // CELL_H
