#ifndef NODE_H
#define NODE_H
#include "segment.h"
#include "graphics.h"
#include "packet.h"
#include "viewport.h"
#include <set>
#include <map>

class AreaTree;
class Node
{
public:
    AreaTree* tree;
    bool visited = false;
    std::vector<segment> boundaries;
    std::vector<Node*> adjacents;
public:
    int indexOfSharedSeg(int index);
    Node(std::vector<segment> bounds,AreaTree *tree);
    Node();
    Vec2d centroid();
    void save(std::ostream& strm,std::map<Node*,int>&nodeMap);
    void load(std::istream& strm, std::map<int,Node*>&nodeMap);
    void draw(mssm::Graphics&g, Viewport view);
    void clearVisited();
    void recursiveDraw(mssm::Graphics&g);
    void visitStuff(std::set<Node*>&visited,std::ostream &strm);
    bool pointInsideThisNode(Vec2d point);
    int indexOfClosestSeg(Vec2d point);
    std::vector<Vec2d> boundaryPoints();
    packet findClosestNode(Vec2d point);
    double distanceToPoint(Vec2d point, Vec2d& closest);
    mssm::Color color = mssm::BLACK;
    void resetColor(mssm::Graphics&g);
    void usePointInside(Vec2d point);
    bool navigateTo(Node* end,std::vector<Node*>&list);
    std::vector<segment> sharedSegs(Node* adj);
    std::vector<segment*> sharedSegsP(Node* adj);
    bool isAdjacentTo(Node* n2);
    void replaceNode(Node* old, Node* newNode);
    void deleteNode(Node* target);
    bool pointOnBound(Vec2d point,double& t, int& index);
    bool inQ = false;
    void visitNodes(std::function<void(Node*)> fn,std::set<Node*>&visited);
    bool doesSegCollide(Vec2d p1, Vec2d p2,double botWidth);
    ~Node();
};

template<class POINT>
bool pointInPolygon(const POINT& p, const std::vector<POINT>& poly)
{
    unsigned int crossCount = 0;
    POINT s1 = poly.back();
    for (const POINT& s2 : poly) {
        if ((s1.y >= p.y && s2.y < p.y) || (s1.y < p.y && s2.y >= p.y)) {
            auto x = ((p.y - s1.y)*(s2.x-s1.x)/(s2.y-s1.y)) + s1.x;
            if (x < p.x) {
                crossCount++;
            }
        }
        s1 = s2;
    }
    return crossCount % 2;
}


#endif // NODE_H
