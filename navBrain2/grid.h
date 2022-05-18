#ifndef GRID_H
#define GRID_H
#include "cell.h"
#include <vector>
#include "graphics.h"
#include "viewport.h"

class grid
{
public:
    //offset of the grid from the left (world x coord of left vertices)
    double offsetX;
    //offset of the grid from the top (world y coord of left vertices)
    double offsetY;
    ///side length of each cell, in inches
    double cellWidth;
    std::vector<std::vector<cell*>> cells;
public:
    grid(int width,int height,double offsetX, double offsetY,double cellWidth); //width and height should be size of grid in squares.
    std::vector<cell> navigate(Vec2d start, Vec2d end, mssm::Graphics& g, Viewport view);
    bool pointInCell(cell* c, Vec2d p);
    //find cell by world coordinates
    cell* findCellByPoint(Vec2d p);
};

#endif // GRID_H
