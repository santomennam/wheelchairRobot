#include "node.h"
#include "packet.h"
#include <iostream>
#include <exception>
#include <set>
#include "areatree.h"
#include "geometry.h"
#include "physics.h"
using namespace std;
using namespace mssm;
int Node::indexOfSharedSeg(int index)
{
    Node* other = adjacents[index];
    for(int i = 0; i < other->boundaries.size(); i++)
    {
        if(other->boundaries[i].p1 == boundaries[index].p2 &&other->boundaries[i].p2 == boundaries[index].p1)
        {
            return i;
        }
    }
    return -1;
}

Node::Node(vector<segment> bounds,AreaTree *tree)
    :tree{tree},boundaries{bounds}
{
    tree->nodes.insert(this);
    adjacents.resize(bounds.size());
    if(!bounds.size())
    {
        throw new logic_error("0 sized boundaries :/");
    }
    // cout<<"node created"<<endl;
}

Node::Node()
{

}

Vec2d Node::centroid()
{
    Vec2d sum;
    if(boundaries.size())
    {
        for(int i = 0; i<boundaries.size();i++)
        {
            sum = sum + boundaries[i].p2;

        }
        return(sum*(1.0/(boundaries.size())));
    }
    return {-100,-100};
}

void Node::save(ostream& strm,map<Node*,int>&nodeMap)
{
    strm<<boundaries.size()<<endl;
    for(int i =0; i<adjacents.size();i++)
    {
        strm<<nodeMap[adjacents[i]]<<" ";
    }
    strm<<endl;
    for(auto seg : boundaries)
    {
        seg.save(strm);
    }
}

void Node::load(istream &strm, std::map<int, Node *> &nodeMap)
{
    int numBounds;
    strm>>numBounds;
    cout<<"numBounds: "<<numBounds<<endl;
    for (int i =0;i<numBounds;i++) {
        int adjIndex;
        strm>>adjIndex;
        adjacents.push_back(nodeMap[adjIndex]);
        cout<<"adjIndex "<<i<<" "<<adjIndex;
        segment seg;
        boundaries.push_back(seg);
    }
    for (int i =0;i<boundaries.size();i++) {
        boundaries[i].load(strm);
    }

}

void Node::draw(mssm::Graphics &g, Viewport view)
{
    for(auto i:boundaries)
    {
        if(!i.open)
        {
            i.C = RED;
        }
        else{
            i.C = BLUE;
        }
        g.line(view.worldToScreen(i.p1),view.worldToScreen(i.p2),i.C);
    }
    g.point(view.worldToScreen(centroid()),color);
}

void Node::clearVisited()
{
    visited = false;
    for(auto n: adjacents)
    {
        if(n&&n->visited)
        {
            n->clearVisited();
        }
    }
}
//write function that takes two nodes that we assume are adjacent on one edge. step one: find out which segment in each node is the one that is shared (find a segment where p1 in this node = p2 in the other and vice versa) -> once find those two indices, set the two node's adjacents(index) = the other node
void Node::visitStuff(std::set<Node *> &visited,std::ostream &strm)
{
    visited.insert(this);

    strm<<this<<" adjacents: ";
    for(Node* i : adjacents)
    {
        strm<<i<<" ";
    }
    //beginning of combining 0x10e5d58 and 0x10e5d58 , this = 0x27d3f6a8
    strm<<" # "<<endl;
    for(Node* n : adjacents)
    {
        if(n && (visited.find(n) == visited.end()))
        {
            n->visitStuff(visited,strm);
        }
    }

}

bool Node::pointInsideThisNode(Vec2d point)
{
    return pointInPolygon(point,boundaryPoints());
}

int Node::indexOfClosestSeg(Vec2d point)
{ //need to make it so its not based on endpoints ugh
    // because otherwise it will draw out of
    Vec2d garbage;
    segment segToReturn({0,0},{0,0},false);
    double closestDist = numeric_limits<double>::max();
    for(int i =0; i<boundaries.size();i++)
    {
        segment b = boundaries[i];
        segment v{point,b.p1,true};
        double cross = crossSeg(v,b);
        if(b.distanceToPoint(point,garbage)<closestDist&&cross<0&&adjacents[i]==nullptr)
        {
            segToReturn = b;
            closestDist = b.distanceToPoint(point,garbage);
        }
    }
    auto found = find(boundaries.begin(),boundaries.end(),segToReturn);
    return(distance(boundaries.begin(),found));
}
//factor in the dot product - only consider those segments that the point is to the right of
double Node::distanceToPoint(Vec2d point, Vec2d&closest)
{
    //cout<<"in distance"<<endl;
    if(pointInsideThisNode(point))
    {
        //cout<<"point inside"<<endl;
        return 0;
    }
    double returning = INT_MAX;
    for(auto s : boundaries)
    {
        Vec2d tmpClosest;
        double d = s.distanceToPoint(point, tmpClosest);
        if(abs(d)<0.00001)
        {
            //cout<<"on the line - in distanceToNode"<<endl;
        }
        if(returning > d)
        {
            returning = d;
            closest = tmpClosest;
        }
    }
    return returning;
}

void Node::resetColor(Graphics &g)
{
    color = WHITE;
}

void Node::usePointInside(Vec2d point)
{

}
Vec2d unit (Vec2d vec)
{
    return (vec*(1/vec.magnitude()));
}


bool Node::navigateTo(Node *end,vector<Node*>&listed)
{
    visited = true;
    if(this != end)
    {
        //shoot out a beam
        //try to find the closest route to direct - make sure im not too close to a wall
        //need a ray to find out which segment we hit and where on tha tsegment we hit
        vector<Node*> list= adjacents;
        std::sort(list.begin(),list.end(),[this,end](Node* a, Node* b) { //check if node exists in tree array
            if(a&&b){
                Vec2d va = (a->centroid()-centroid()).unit();
                Vec2d vb = (b->centroid()-centroid()).unit();
                Vec2d vend = (end->centroid()-centroid()).unit();
                return va*vend > vb*vend;
            }
            if(a&&!b)
            {
                return false;
            }
            if(!a&&b)
            {
                return true;
            }
            return false;
        });
        // std::reverse(list.begin(),list.end());
        for(int i = 0; i < list.size();i++)
        {
            auto n = list[i];
            if(n&&!n->visited&&sharedSegs(n)[0].open) //make node findAdjacent method
            {
                // cout<<"5 probably not handling this right bc potentially more than one segment between these two nodes because of splitEdge"<<endl;
                if(n->navigateTo(end,listed))
                {
                    listed.push_back(this);
                    return true;
                }
            }
        }
    }
    else{
        listed.push_back(this);
        return true;
    }
    return false;
}

vector<segment> Node::sharedSegs(Node *adj)
{
    vector<segment> returner;
    //' cout<<"6 probably not handling this right bc potentially more than one segment between these two nodes because of splitEdge"<<endl;
    for(int i = 0; i < adjacents.size(); i++)
    {
        if(adjacents[i] == adj)
        {
            returner.push_back(boundaries[i]);
        }
    }
    // segment returner({INT_MAX,INT_MAX},{INT_MIN,INT_MIN},true);
    return returner;
}

std::vector<segment *> Node::sharedSegsP(Node *adj)
{
    vector<segment*> returner;
    //' cout<<"6 probably not handling this right bc potentially more than one segment between these two nodes because of splitEdge"<<endl;
    for(int i = 0; i < adjacents.size(); i++)
    {
        if(adjacents[i] == adj)
        {
            returner.push_back(&boundaries[i]);
        }
    }
    // segment returner({INT_MAX,INT_MAX},{INT_MIN,INT_MIN},true);
    return returner;
}

bool Node::isAdjacentTo(Node *n2)
{
    for(int i = 0; i < adjacents.size(); i++)
    {
        if(adjacents[i] == n2)
        {
            return true;
        }
    }
    return false;
}
void Node::deleteNode(Node* target)
{
    if(this)
    {
        if(this==target)
        {
            delete this;
            return;
        }
        for(auto n: adjacents)
        {
            if(n&&!n->visited)
            {
                n->deleteNode(target);
            }
        }
    }
}

bool Node::pointOnBound(Vec2d point, double &t, int &index)
{
    Vec2d dummy{0,0};
    for(int i =0; i<boundaries.size();i++)
    {
        auto n=boundaries[i];
        if(abs(n.distanceToPoint(point,dummy,t))<0.001)
        {
            index = i;
            return true;
        }
    }
    index = -1;
    return false;
}

void Node::visitNodes(std::function<void (Node *)> fn, std::set<Node *> &visited)
{
    visited.insert(this);

    for(Node* n : adjacents)
    {
        if(n && (visited.find(n) == visited.end()))
        {
            n->visitNodes(fn, visited);
        }
    }
    fn(this);

}

bool Node::doesSegCollide(Vec2d p1, Vec2d p2, double botWidth)
{
    for(auto s: boundaries)
    {
        if(!s.open)
        {
            if(distanceBetweenSegsSquared(p1,p2,s.p1,s.p2) < botWidth)
            {
                return true;
            }
        }
    }
    return false;
}

Node::~Node()
{
    if(tree){
        tree->nodes.erase(this);
        cout<<"node deleted!"<<endl;
    }
}

void Node::replaceNode(Node *old, Node *newNode)
{
    // cout<<"in replace, this: "<<this<<endl;
    for(int i = 0; i <adjacents.size(); i++)
    {
        if(adjacents[i] == old)
        {
            adjacents[i] = newNode;
        }
    }
    //look through adjacents, change from pointing to old to new
}
packet Node::findClosestNode(Vec2d point)
{
    //issues here, with child maybe??
    Vec2d closest;
    visited = true;
    packet toReturn;
    toReturn.node = this;
    toReturn.distance = distanceToPoint(point,closest);
    //    if(abs(toReturn.distance)<0.00001)
    //    {
    //      //  cout<<"DISTANCE OF ZERO!!!!!"<<endl;
    //    }
    toReturn.closestOnNode = closest;
    for(auto n: adjacents)
    {
        if(n&&!n->visited)
        {
            packet child = (n->findClosestNode(point));
            if(child.distance < toReturn.distance)
            {
                toReturn = child;
            }
        }

    }
    return(toReturn);
}
std::vector<Vec2d> Node::boundaryPoints()
{
    vector<Vec2d> boundPoints;
    for(auto s:boundaries)
    {
        boundPoints.push_back(s.p1);
    }
    if(!boundaries.size())
    {
        throw new std::logic_error("empty boundaries");
    }
    return boundPoints;
}


