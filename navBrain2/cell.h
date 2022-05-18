#ifndef CELL_H
#define CELL_H
#include "graphics.h"

class grid;
class cell
{
public:
    grid* parentGrid;
    Vec2d centroid; //in world coords
    bool visited = false;
    bool blocked = false;
    mssm::Color color = mssm::BLACK;
public:
    cell(grid* grid,Vec2d centroid);
    double distanceToPoint(Vec2d point, Vec2d& closest);
    bool pointInCell(Vec2d point);
};

#endif // CELL_H
