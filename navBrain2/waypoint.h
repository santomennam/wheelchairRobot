#ifndef WAYPOINT_H
#define WAYPOINT_H
#include <iostream>
#include "cell.h"
#include "graphics.h"
#include "viewport.h"
#include "node.h"

class Waypoint{
public:
    cell* c;
    double cost; //sum of previous costs - take previous cost and add distance from centroid to centroid
    Waypoint* previous;
    double hueristic; //straight-line
public:
    Waypoint(cell* n,Waypoint* prev, Vec2d destination);
    double totalCost() {return cost+hueristic;}
    bool containsCell(cell* c);
    std::vector<cell*> extract();
    std::vector<Vec2d> pathPoints(double botWidth);
    void recalculateCost(Vec2d destination);

};

//class Waypoint{
//public:
//    Node* node;
//    double cost; //sum of previous costs //take previous cost and add distance from centroid to centroid
//    Waypoint* previous;
//    double hueristic; //straight-line
//public:
//    Waypoint(Node* n,Waypoint* prev, Vec2d destination);
//    double totalCost() {return cost+hueristic;}
//    bool containsNode(Node* n);
//    std::vector<Node*> extract();
//    std::vector<Vec2d> pathPoints(double botWidth);
//    void recalculateCost(Vec2d destination);

//};

bool operator<(const Waypoint& w1, const Waypoint& w2);
#endif // WAYPOINT_H
