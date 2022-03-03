#ifndef AREATREE_H
#define AREATREE_H
#include "graphics.h"
#include "node.h"
#include "segment.h"
#include "packet.h"
#include "viewport.h"
#include "waypoint.h"
#include "physics.h"
//merge adjacent triangles toreduce clutter - have to meet certain standards - roughly parallel bases, only if the ends are both closed
class AreaTree
{
public:
    std::set<Node*> nodes;
    Node* head;
    double botWidth;
public:
    void clear();
    void showAdjacents(mssm::Graphics&g,Viewport view);
    AreaTree(double width, double height,double botWidth);
    void save(std::ostream& strm);
    void load(std::istream& strm);
    void twoPreexisting(Vec2d point);
    void attachNode(std::vector<segment>bounds, Node* location, int index); //index determines which side to attach new node to
    void tetherNode(Node* n1, Node*location, int index);
    void splitNode(Vec2d point, Node* node);
    void visitStuff(std::ostream &strm);
    void pointOutNode(Vec2d point, Node* node);
    void draw(mssm::Graphics& g, Viewport view);
    void drawTree(mssm::Graphics& g);
    void findAdjacents(Node* n1, Node* n2);
    packet findClosestNode(Vec2d point);
    void placer(Vec2d point); //combine a whole bunch of methods here to find where the point should draw itself
    void resetColor(mssm::Graphics&g);
    std::vector<Vec2d> navigation(Vec2d start, Vec2d end,mssm::Graphics&g,Viewport view);
    Node* combineNodes(Vec2d p1, Vec2d p2);
    Node* combineNodes(Node* n1, Node* n2);
    void replaceNode(Node* old, Node* newNode);
    void dumpToFile(std::ostream& strm);
    void splitEdge(Node* closestNode,double t,int index);
    bool isNodeInTree(Node* a);
    void visitNodes(std::function<void(Node*)> fn);
    bool hasNodeBeenDeleted(Node a);
    Waypoint* aStar(Vec2d start, Vec2d destination,mssm::Graphics&g,Viewport view);
    bool doesSegmentCollide(Vec2d p1, Vec2d p2);
    void shortenPath(Waypoint* current,Vec2d destination);

};

#endif // AREATREE_H
