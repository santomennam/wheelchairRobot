#include "cell.h"
#include "grid.h"


cell::cell(Grid* grid,Vec2d centroid, Vec2d indices)
    :parentGrid{grid}, centroid{centroid}, indices{indices}
{}

double cell::distanceToPoint(Vec2d point, Vec2d &closest)
{
    return 0;
}

bool cell::pointInCell(Vec2d point)
{
    return parentGrid->pointInCell(this, point);
}
