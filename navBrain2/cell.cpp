#include "cell.h"
#include "grid.h"


cell::cell(grid* grid,Vec2d centroid)
    :parentGrid{grid}
{}

double cell::distanceToPoint(Vec2d point, Vec2d &closest)
{
    return 0;
}

bool cell::pointInCell(Vec2d point)
{
    return parentGrid->pointInCell(this, point);
}
