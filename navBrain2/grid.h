#ifndef GRID_H
#define GRID_H
#include "cell.h"
#include <vector>
#include "graphics.h"
#include "viewport.h"
#include "waypoint.h"
#include "graphics.h"
#include "viewport.h"

class Grid
{
public:
    //offset of the grid from the left (world x coord of left vertices)
    double offsetX;
    //offset of the grid from the top (world y coord of left vertices)
    double offsetY;
    ///side length of each cell, in inches
    double cellWidth;
    std::vector<std::vector<cell*>> cells;
    int width;
    int height;
    double botWidth;
public:
    std::vector<Vec2d> navigation(Vec2d start, Vec2d end, mssm::Graphics& g, Viewport view);
    std::vector<cell*> getAdjacentCells(cell* c);
    bool areCellsAdjacent(cell* c1, cell* c2);
    void draw(mssm::Graphics& g, Viewport view);
    void shortenPath(Waypoint* current, Vec2d destination);
    Grid(int width,int height,double robotWidth, double offsetX, double offsetY,double cellWidth); //width and height should be size of grid in squares.
    bool pointInCell(cell* c, Vec2d p);
    //find cell by world coordinates
    cell* findCellByPoint(Vec2d p);
    Waypoint* aStar(Vec2d start, Vec2d destination,mssm::Graphics&g,Viewport view);
    //take a cell by indices and see if all cells within radius (in inches) are open
    bool isClear(Vec2i32 indices, double radius);
    //apply a function to all cells in the grid
    void applyToCells(std::function<void (cell *)> fn);
};

#endif // GRID_H
