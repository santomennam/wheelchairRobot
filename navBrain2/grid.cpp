#include "grid.h"
#include <set>
#include <functional>
#include <queue>
#include <stdio.h>
#include <sysinfoapi.h>
using namespace std;
using namespace mssm;

//width and height should be size of grid in squares.
std::vector<Vec2d> Grid::navigation(Vec2d start, Vec2d end, mssm::Graphics &g, Viewport view)
{
    std::vector<Vec2d> path;
    long long int begin = GetTickCount64();
    Waypoint* pathWay = aStar(start,end,g,view);
    long long int endAStar = GetTickCount64();
    if(pathWay)
    {
        path = pathWay->extractToVec2d();
    }
    long long int endTime = GetTickCount64();
    double elapsed = (endTime - begin)*1e-3;
    double elapsedAStar =(endAStar - begin)*1e-3;
    printf("A*: %.3f seconds to find a path with %i nodes\n", elapsedAStar,path.size());
    printf("Time measured: %.3f seconds to find a path with %i nodes\n", elapsed,path.size());
    return path;
}

std::vector<cell *> Grid::getAdjacentCells(cell *c)
{
    vector<cell*> returner;
    Vec2i32 indices = c->indices;
    for(int i = indices.x -1; i <indices.x+2; i++)
    {
        for(int j = indices.y-1; j <indices.y+2; j++)
        {
            if(i < width && j <height && i > 0 && j > 0)
            {
                returner.push_back(cells[i][j]);
            }
        }
    }
    return returner;
}

bool Grid::areCellsAdjacent(cell *c1, cell *c2)
{
    return (abs(c1->indices.x-c2->indices.x)== 1 && abs(c1->indices.y-c2->indices.y)== 1);
}

void Grid::draw(mssm::Graphics &g, Viewport view)
{
    for(int i = 0; i <height; i ++)
    {
        g.line(view.worldToScreen({offsetX,(offsetY+i*cellWidth)}),view.worldToScreen({offsetX+width*cellWidth,(offsetY+i*cellWidth)}));
    }
    for(int i =0; i < width; i ++)
    {
        g.line(view.worldToScreen({offsetX+i*cellWidth,offsetY}),view.worldToScreen({offsetX+i*cellWidth,(offsetY+height*cellWidth)}));
    }
    for(auto &col:cells)
    {
        for(cell* c: col)
        {
            if(c->drawMe){
                g.rect(view.worldToScreen(c->centroid-Vec2d{cellWidth,cellWidth}),cellWidth*view.scale,cellWidth*view.scale,WHITE);
                //       g.ellipse(view.worldToScreen(c->centroid),3*view.scale,3*view.scale,c->color);
            }
        }
    }
}

void Grid::shortenPath(Waypoint *current, Vec2d destination)
{
    cout << "ShortenPath not implemented" << endl;
    //        if(current->previous&&current->previous->previous)
    //        {
    //            if(!doesSegmentCollide(current->c->centroid,current->previous->previous->c->centroid))
    //            {
    //                // cout<<"straightening"<<endl;
    //                current->previous = current->previous->previous;
    //                current->recalculateCost(destination);
    //            }
    //        }
}

Grid::Grid(int width, int height, double robotWidth, double offsetX, double offsetY, double cellWidth)
    :cellWidth{cellWidth}, botWidth{robotWidth}, offsetX{offsetX}, offsetY{offsetY}, width{width}, height{height}
{
    for(int i = 0; i < width; i ++){
        cells.push_back(vector<cell*>());
        for(int j = 0; j < height; j++)
        {
            cell* c = new cell(this,{(i*cellWidth+offsetX),(j*cellWidth+offsetY)},{i,j});
            cells[i].push_back(c);
        }
    }
}


bool Grid::pointInCell(cell *c, Vec2d p)
{
    return findCellByPoint(p) == c;
}

cell* Grid::findCellByPoint(Vec2d p)
{
    p = p -Vec2d{offsetX,offsetY};
    int x = p.x/cellWidth;
    int y = p.y/cellWidth;
    if(x < width && y < height) //might need to be <=
    {
        return cells[x][y];
    }
    return nullptr;
}

Waypoint *Grid::aStar(Vec2d start, Vec2d destination, mssm::Graphics &g, Viewport view)
{
   // return nullptr;
    //  traverse tree and reset inQ with visitStuff()
    applyToCells([](cell* c){c->inQ = false; c->color = BLACK;});
    cell* startCell = findCellByPoint(start);
    cell* endCell = findCellByPoint(destination);
    startCell->color = RED;
    endCell->color = PURPLE;
    if(startCell == endCell)
    {
        cout<<"start and end are in the same cell!"<<endl;
        return  nullptr;
    }
    priority_queue<Waypoint*,vector<Waypoint*>,function<bool(const Waypoint*,const Waypoint*)>> q([](const Waypoint* w1, const Waypoint* w2){return *w2<*w1;});
    Waypoint* first = new Waypoint(startCell,nullptr,destination);
    q.push(first);
    first->c->inQ = true;
    while(q.size())
    {
        //look for area 2 back  `
        //prev of prev
        //if can draw ray from here to there, this->prev = prev->prev
        g.clear();
        draw(g,view);
        // cout<<"Plotting..."<<endl;
        Waypoint* current = q.top();
        // g.ellipseC(view.worldToScreen(current->node->centroid()),10,10,mssm::CYAN,mssm::CYAN); //DRAWINGGGGGGGGGGGGGGGGGGGG
        // g.polyline(view.worldToScreen(current->pathPoints(botWidth)),mssm::WHITE);
        // g.draw();
        q.pop();
        // current->node->inQ = false;
        if(areCellsAdjacent(current->c,endCell))
        {
            Waypoint* path = new Waypoint(endCell,current,destination);
         //   shortenPath(path,destination);
            return path;
        }
        auto currentAdjacents = getAdjacentCells(current->c);
        for (int i =0; i < currentAdjacents.size(); i++)
        {
            cell* c = currentAdjacents[i];
            if(isClear(c->indices,botWidth)&&!c->inQ)
            {
              //  shortenPath(current,destination);
                Waypoint* adj = new Waypoint(c,current,destination);
                // maybe need vector of previous running in tandem: different threads could not have visited nodes in their previous and go infinitely
                q.push(adj);
                c->inQ = true;
            }
        }
    }
    cout<<"No path"<<endl;
    return  nullptr;
}

bool Grid::isClear(Vec2i32 indices, double radius)
{
    int cellRadius = radius/cellWidth+1; //number of cells along radius
    for(int i = indices.x-cellRadius; i < indices.x+cellRadius; i++)
    {
        for(int j = indices.y-cellRadius; j < indices.y+cellRadius; j++)
        {
            if(i < width && j < height && i > 0 && j > 0)
            {
                if(cells[i][j]->blocked)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

void Grid::applyToCells(std::function<void (cell *)> fn)
{
    for(auto col : cells)
    {
        for(cell* c : col)
        {
            fn(c);
        }
    }
}
