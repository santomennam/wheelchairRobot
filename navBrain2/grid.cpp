#include "grid.h"

//width and height should be size of grid in squares.
grid::grid(int width, int height, double offsetX, double offsetY, double cellWidth)
    :cellWidth{cellWidth}, offsetX{offsetX}, offsetY{offsetY}
{
    for(int i = 0; i < width; i ++){
        for(int j = 0; j < height; j++)
        {
            cell* c = new cell(this,{(i*cellWidth+offsetX),(j*cellWidth+offsetY)});
            cells[i][j] = c;
        }
    }
}

std::vector<cell> grid::navigate(Vec2d start, Vec2d end, mssm::Graphics& g, Viewport view)
{

}

bool grid::pointInCell(cell *c, Vec2d p)
{
    return findCellByPoint(p) == c;
}

cell* grid::findCellByPoint(Vec2d p)
{
    int x = p.x/cellWidth;
    int y = p.y/cellWidth;
    return cells[x][y];
}
