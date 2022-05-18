#include "cell.h"

cell::cell(grid* grid)
    :parentGrid{grid}
{}

double cell::distanceToPoint(Vec2d point, Vec2d &closest)
{

}

bool cell::pointInCell(Vec2d point)
{
    return parentGrid->pointInCell(this, point);
}
