#ifndef GRID_H
#define GRID_H
#include "cell.h"
#include <vector>

class grid
{
public:
    std::vector<std::vector<cell>> cells;
public:
    grid(int width,int height,double squareWidth); //width and height should be size of grid in squares.

};

#endif // GRID_H
