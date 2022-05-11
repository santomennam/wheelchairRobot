#include <iostream>
#include "areatree.h"
#include <assert.h>
#include <set>
#include <functional>
#include <queue>
#include <vector>
using namespace std;
using namespace mssm;

void AreaTree::clear()
{
    for(auto& node : nodes)
    {
        node->tree = nullptr;
        delete node;
    }
    nodes.clear();
    head = nullptr;
}

void AreaTree::showAdjacents(Graphics &g, Viewport view)
{
    for(auto& node : nodes)
    {
        for(int i =0; i<node->adjacents.size();i++)
        {
            if(node->adjacents[i])
            {
                g.line(view.worldToScreen(node->centroid()),view.worldToScreen(node->adjacents[i]->centroid()));
            }
        }
    }
}

AreaTree::AreaTree(double width, double height,double Width)
    :botWidth{Width}
{
    double cellSize = 2000;//botWidth*2;
    int numRows = 1;//width/cellSize;
    int numCols = 1;//height/cellSize;
    double offx = -cellSize/2;//-width/2; ////
    double offy = -cellSize/2;//-height/2; //
    vector<vector<Node*>> grid(numRows, vector<Node*>(numCols));

    for (int r = 0; r < numRows; r++) {
        double top = offy+(r+1)*cellSize;
        double bottom = offy+r*cellSize;
        for (int c = 0; c < numCols; c++) {
            double right = offx+(c+1)*cellSize;
            double left = offx+c*cellSize;
            vector<segment> segs{
                { {top, right}, {top, left}, botWidth},
                { {top, left}, {bottom, left}, botWidth},
                { {bottom, left}, {bottom, right}, botWidth},
                { {bottom, right}, {top, right}, botWidth},
            };
            grid[r][c] = new Node(segs,this);
        }
    }

    for (int r = 0; r < numRows-1; r++) {
        for (int c = 0; c < numCols-1; c++) {
            findAdjacents(grid[r][c], grid[r][c+1]);
            findAdjacents(grid[r][c], grid[r+1][c]);
        }
    }

    head = grid[0][0];
    vector<segment> segs{{{width,-height},{width,height}, botWidth},{{width,height},{-width,height},botWidth},{{-width,height},{width,-height},botWidth}};
    //head = new Node{segs,this};
    vector<segment>segs2{{{width,-height},{-width,height},botWidth},{{-width,height},{-width,-height},botWidth},{{-width,-height},{width,-height},botWidth}};
    // Node *node = new Node(segs2,this);
    //findAdjacents(node,head);
}

void AreaTree::save(ostream& strm)
{
    map<Node*,int> nodeMap;
    int i =1;
    nodeMap[nullptr] =0;
    for(auto& node : nodes)
    {
        nodeMap[node]=i;
        i++;
    }
    strm<<to_string(nodes.size())<<endl;
    for(auto& node : nodes)
    {
        node->save(strm,nodeMap);
    }
}

void AreaTree::load(istream &strm)
{
    clear();
    int numNodes;
    strm>>numNodes;
    map<int,Node*> nodeMap;
    nodeMap[0] =nullptr;
    for(int i =0; i<numNodes; i++)
    {
        nodeMap[i+1] = new Node();
        nodes.insert(nodeMap[i+1]);
    }
    head = nodeMap[1];

     for(int i =0; i<numNodes; i++)
    {
        nodeMap[i+1]->load(strm,nodeMap);
    }
}


packet AreaTree::findClosestNode(Vec2d point)
{
    head->clearVisited();
    packet toReturn = head->findClosestNode(point); //leave
    return toReturn;
}

void AreaTree::placer(Vec2d point) //leave
{
    placedPoints.push_back(point);
    // visitStuff(cout);
    double t = -1;
    int index = 0;
    packet closestPacket = findClosestNode(point);
    Node* closestNode = closestPacket.node;
    if(closestNode->pointOnBound(point,t,index))
    {
        //call areaTree splitEdge(Node* closestNode,t,index)
        //another area on other side? also split
        //update boundaries and adjacents
        //draw pictures!!!
        //find point along way
        //split the boundaries
        //split boundary in adjacents
        //fix adjacents
        splitEdge(closestNode,t,index); //hmm
    }

    else if(closestNode->pointInsideThisNode(point)) //problem
    {
        splitNode(point,closestNode);

    }
    else {
        resetTree();
        return;
      //  pointOutNode(point,closestNode);
    }
}

void AreaTree::resetColor(Graphics &g)
{
    for(auto n :nodes)
    {
        if(n)
        {
            n->resetColor(g);
        }

    }
}

vector<Vec2d> AreaTree::navigation(Vec2d start, Vec2d end, Graphics&g,Viewport view)
{
   // cout<<"nav2"<<endl;

    //traversal: take starting points, get two areas.
    //find path between areas

    //traversal:

    packet startPack = findClosestNode(start);
    packet endPack = findClosestNode(end);

    Node *startNode = startPack.node;
    startNode->color = RED;
    Node *endNode = endPack.node;
    endNode->color = PURPLE;
    if(startNode == endNode)
    {
        cout<<"start and end are in the same node!"<<endl;
    }
    vector<Node*> path;
    head->clearVisited();
    vector<Vec2d> pointsReturn;
    Waypoint* use = aStar(start,end,g,view);
    if(use)
    {
        pointsReturn = use->pathPoints(botWidth);
    }

    //modes: explore, destination, manual - should be able to add stored waypoints to navigate to
    //find list of areas
    //if area tree gets updated - find a wall or something, its time to re calculate the path
    //create a list of waypoints
    //give it start and endpoints, return with list of waypoints
    //could be midpoint of sides of the area, or middle of the area, or both
    return pointsReturn;
}

Node* AreaTree::combineNodes(Vec2d p1, Vec2d p2)
{
    packet packet1 = findClosestNode(p1);
    Node* node1 = packet1.node;
    packet packet2 = findClosestNode(p2);
    Node* node2 = packet2.node;
    return combineNodes(node1, node2);
}
vector<int> getNonADjacentIndices(std::vector<segment> segs, vector<segment> shared)
{
    vector<int> indices;
    bool hitShared = false;
    int firstIndex{0};
    for(int i = 0; i < segs.size(); i++){
        //go sounterclockwise starting after shared segment(s)
        auto it = find(shared.begin(),shared.end(),segs[i]);
        if(it != shared.end()) {
            hitShared = true;
        }
        else if (hitShared) {
            //yay! we're golden
            firstIndex = i;
            break;

        }
        //firstIndex is now set correctly - if not reset from zero, it must be zero

    }
    for(int i =0; i<(segs.size()-shared.size()); i++)
    {
        indices.push_back((firstIndex + i) % segs.size());
    }
    return indices;
}

Node* AreaTree::combineNodes(Node* n1, Node* n2)
{
    //cout<<"beginning of combining "<<n1<<" and "<<n2<<" , this = "<<this<<endl;
    //uff(cout);
    if(n1->boundaries.empty() || n2->boundaries.empty())
    {
        cout<<"what the hell"<<endl;
    }
    //return new node if created, otherwise return null
    //if return a new node, should be added into the list of auto-merged.

    if(n1==n2)
    {
        return nullptr;
    }
    if(!n1->isAdjacentTo(n2))
    {
        cout<<"not adjacent!"<<endl;

        return nullptr;
    }
    vector<segment> shared = n1->sharedSegs(n2);
    //cout<<"3 probably not handling this right bc potentially more than one segment between these two nodes because of splitEdge"<<endl;

    vector<Node*> adjacents;
    vector<segment> segs;
    vector<int> indices1 = getNonADjacentIndices(n1->boundaries,shared);
    for(int i =0; i<indices1.size(); i++)
    {
        adjacents.push_back(n1->adjacents[indices1[i]]);
        segs.push_back(n1->boundaries[indices1[i]]);
    }
    shared = n2->sharedSegs(n1);
    //cout<<"4 probably not handling this right bc potentially more than one segment between these two nodes because of splitEdge"<<endl;

    vector<int> indices2 = getNonADjacentIndices(n2->boundaries,shared);
    for(int i =0; i<indices2.size(); i++)
    {
        adjacents.push_back(n2->adjacents[indices2[i]]);
        segs.push_back(n2->boundaries[indices2[i]]);
    }
    //concavity test
    for(int i =0; i<segs.size();)
    {

        double cp = crossSeg(segs[i], segs[((i+1)%segs.size())]);
        if(cp <= 0)
        {
            //cout<<"created space would not be convex"<<endl;
            return nullptr;
        }
        else{
            i++;
        }
    }

    //from here on out, assume convex
    //line up adjacents
    //delete two other nodes

    Node* newNode = new Node(segs,this);
    for(int i = 0; i <adjacents.size();i++)
    {
        newNode->adjacents[i] = adjacents[i];
    }

    replaceNode(n1,newNode);
    replaceNode(n2,newNode);
    cout<<"end of combining "<<n1<<" and "<<n2<<endl;
    //uff(cout);
    return newNode;

}
void AreaTree::replaceNode(Node *old, Node *newNode)
{
    cout<< "replacing "<<old<<" with "<<newNode<<endl;
    // uff(cout);
    for(auto n : nodes)
    {
        n->replaceNode(old,newNode);
    }
    if(old == head)
    {
        head = newNode;
    }
    //cout<<old<<endl;
    //insert sanity check
    if(isNodeInTree(old))
    {
        cout<<"UH OH! NODE IN TREE AFTER DELETION!"<<endl;
        throw new logic_error("UH OH! NODE IN TREE AFTER DELETION!");
    }
    delete old;


}

void AreaTree::splitEdge(Node *closestNode, double t, int index)
{
    segment edge = closestNode->boundaries[index];
    Vec2d newPoint = edge.p1+((edge.p2-edge.p1)*t);
    segment e1(edge.p1,newPoint,botWidth);
    segment e2(newPoint,edge.p2,botWidth);
    Node* other = closestNode->adjacents[index];
    if(other)
    {
        int otherIndex = closestNode->indexOfSharedSeg(index);
        other->boundaries[otherIndex] = e1;
        other->boundaries.insert(other->boundaries.begin()+otherIndex+1,e2);
        other->adjacents.insert(other->adjacents.begin()+otherIndex+1,closestNode);
    }
    closestNode->boundaries[index] = {e2.p2,e2.p1,botWidth}; //sketchy
    closestNode->boundaries.insert(closestNode->boundaries.begin()+index+1,{e1.p2,e1.p1,botWidth});
    closestNode->adjacents.insert(closestNode->adjacents.begin()+index+1,other);
}

bool AreaTree::isNodeInTree(Node* a)
{
    bool found = false;
    visitNodes([&found,a](Node* b){
        if(a==b){
            found = true;
        }});
    return found;
}

void AreaTree::visitNodes(std::function<void (Node *)> fn)
{
    std::set<Node*> visited;
    head->visitNodes(fn,visited);
}
void AreaTree::shortenPath(Waypoint* current,Vec2d destination)
{

    if(current->previous&&current->previous->previous)
    {
        if(!doesSegmentCollide(current->node->centroid(),current->previous->previous->node->centroid()))
        {
           // cout<<"straightening"<<endl;
            current->previous = current->previous->previous;
            current->recalculateCost(destination);
        }
    }
}

vector<Vec2d> findMinMax(vector<Vec2d> points)
{
    double minX = INT_MAX;
    double maxX = 0;
    double minY = INT_MAX;
    double maxY = 0;
    for(auto p : points)
    {
        if(p.x > maxX)
        {
            maxX = p.x;
        }
        if(p.x < minX)
        {
            minX = p.x;
        }
        if(p.x > maxY)
        {
            maxY = p.y;
        }
        if(p.y < minY)
        {
            minY = p.y;
        }
    }

    Vec2d x{maxX,minX};
    Vec2d y{maxY,minY};
    return vector<Vec2d> {x,y};
}

void AreaTree::resetTree()
{
    //find min and max x and y, make somewhat bigger than that
    vector<Vec2d> pointsCopy = placedPoints;
    vector<Vec2d>minMaxes = findMinMax(pointsCopy);
    //now use this information to make a rectangle of width (xMax + l) and height (hMax + h) where l and h are arbitrary
    //then populate with pointsCopy
}
Waypoint* AreaTree::aStar(Vec2d start, Vec2d destination, Graphics &g,Viewport view)
{
    //traverse tree and reset inQ with visitStuff()
    visitNodes([](Node* b){b->inQ = false; b->color = BLACK;});
    packet startPack = findClosestNode(start);
    packet endPack = findClosestNode(destination);
    Node *startNode = startPack.node;
    startNode->color = RED;
    Node *endNode = endPack.node;
    endNode->color = PURPLE;
    if(startNode == endNode)
    {
        cout<<"start and end are in the same node!"<<endl;
        return  nullptr;
    }
    priority_queue<Waypoint*,vector<Waypoint*>,function<bool(const Waypoint*,const Waypoint*)>> q([](const Waypoint* w1, const Waypoint* w2){return *w2<*w1;});
    Waypoint* first = new Waypoint(startNode,nullptr,destination);
    q.push(first);
    first->node->inQ = true;
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
        if(current->node->isAdjacentTo(endNode))
        {           
            Waypoint* path = new Waypoint(endNode,current,destination);
            shortenPath(path,destination);
            return path;
        }

        for (int i =0; i < current->node->adjacents.size(); i++)
        {
            Node* n = current->node->adjacents[i];
            segment& s = current->node->boundaries[i];
            if(n && s.open && !n->inQ && !current->containsNode(n))
            {
               shortenPath(current,destination);
                Waypoint* adj = new Waypoint(n,current,destination);
                // maybe need vector of previous running in tandem: different threads could not have visited nodes in their previous and go infinitely
                q.push(adj);
                n->inQ = true;
            }
        }
    }
    cout<<"No path"<<endl;
    return  nullptr;

}

bool AreaTree::doesSegmentCollide(Vec2d p1, Vec2d p2)
{
    bool collides = false;
    visitNodes([p1,p2,this,&collides](Node* n){if(n->doesSegCollide(p1,p2,botWidth)){collides = true;}} );
    return collides;
}



//bool AreaTree::hasNodeBeenDeleted(Node a)
//{
//    if(find(nodes.begin(),nodes.end(),a) == nodes.end())
//    {
//        return false;
//    }
//    return true;
//}


void AreaTree::visitStuff(std::ostream &strm)
{
    std::set<Node*> visited;
    //this isnt what this was supposed to be used for, however i dont remember what it was going to be so here we go
    strm<<"----------Start of Dump---------"<<endl;
    head->visitStuff(visited,strm);

}
//void AreaTree::twoPreexisting(Vec2d point)
////{
////    vector<segment> tempBounds;
////    tempBounds.push_back({{100,200},{100,100},botWidth});
////    tempBounds.push_back({{200,100},{100,100},botWidth});
////    tempBounds.push_back({{200,100},{100,100},botWidth});
////    segment seg2(tempBounds[0].p1, point,botWidth);
////    vector<segment> segments;
////    if(crossSeg(tempBounds[0], seg2)>0)
////    {
////        segments.push_back(seg2);
////        segments.push_back({point,tempBounds[0].p2,botWidth});
////        segments.push_back(tempBounds[0]);
////        Node node(segments);
////    }
////    else {
////        segments.push_back(tempBounds[0]);
////        segments.push_back({point,tempBounds[0].p2,botWidth});
////        segments.push_back(seg2);
////        Node node(segments);
////    }

////    //do something with created node
//}

void AreaTree::splitNode(Vec2d point, Node *node)
{

    vector<segment> segs = node->boundaries;
    vector<Node*> nodes;
    for(int i = 0; i<node->boundaries.size(); i++)
    {
        segment b = segs[i];
        vector<segment> nodeSegs;
        Node* adjacent = node->adjacents[i];
        segment seg1{point,b.p1,botWidth};
        segment seg2{b};
        segment seg3{b.p2,point,botWidth};
        nodeSegs.push_back(seg1);
        nodeSegs.push_back(seg2);
        nodeSegs.push_back(seg3);
        Node* node1 = new Node(nodeSegs,this);
        nodes.push_back(node1);
        findAdjacents(node1,adjacent);
    }

    for(int i = 0; i <nodes.size(); i++)
    {
        findAdjacents(nodes[i],nodes[(i+1)%nodes.size()]);
    }

    if(head == node)
    {
        if(!nodes[1])
        {
            cout<<"frick"<<endl;
        }
        head = nodes[1];
    }
    if(isNodeInTree(node))
    {
        cout<<"2 UH OH! NODE IN TREE AFTER DELETION!"<<endl;
        throw new logic_error("2 UH OH! NODE IN TREE AFTER DELETION!");
    }
    delete(node);
   // cout<<"Dump in Split"<<endl;
    //visitStuff(cout);
    while(nodes.size())
    {
        for(int j = 0; j < nodes[0]->adjacents.size();j++)
        {
            Node* tempNode = nodes[0];
            Node* adjNode = tempNode->adjacents[j];
            if(tempNode && adjNode)
            {
                Node* newNode = combineNodes(tempNode,adjNode);
                if(newNode)
                {
                    nodes.push_back(newNode);
                    //cout<<"combined"<<endl;
                    auto i = find(nodes.begin(), nodes.end(), adjNode);
                    if(i != nodes.end())
                    {
                        nodes.erase(i);
                    }
                    break;
                }


            }
        }
        nodes.erase(nodes.begin());
    }

}

void AreaTree::pointOutNode(Vec2d point, Node *node)
{
    //cout<<"called"<<endl;
    vector<segment> segs;
    if(!node->boundaries.size())
    {

    }
    segment closest{node->boundaries[node->indexOfClosestSeg(point)]};//oof
    segment seg1{point,closest.p2,botWidth};
    segment seg2{closest.p2,closest.p1,botWidth};
    segment seg3{closest.p1,point,botWidth};
    segs.push_back(seg1);
    segs.push_back(seg2);
    segs.push_back(seg3);
    Node* node1 = new Node{segs,this};
    findAdjacents(node, node1);
    vector<Node*> nodes;
    nodes.push_back(node1);
    while(!nodes.empty())
    {
        Node* n = nodes.back();
        nodes.pop_back();

        for(int j = 0; j < n->adjacents.size();j++)
        {
            if(n->adjacents[j])
            {
                Node* newNode = combineNodes(n,n->adjacents[j]);
                if(newNode)
                {
                    nodes.push_back(newNode);
                    break;
                }
            }
        }
    }
}
void AreaTree::draw(Graphics &g, Viewport view)
{
    //traversal
    if(head){
        for(auto i: nodes)
        {
            if(i)
            {
                i->draw(g,view);
            }
        }

    }
}

void AreaTree::drawTree(Graphics &g)
{

}

//write function that takes two nodes that we assume are adjacent on one edge.
//step one: find out which segment in each node is the one that is shared
//(find a segment where p1 in this node = p2 in the other and vice versa) -> once founnd those two indices,
//set the two node's adjacents(index) = the other node

void AreaTree::findAdjacents(Node *n1, Node *n2)
{
    if(n1&&n2)
    {
        for(auto s1:n1->boundaries)
        {
            for(auto s2:n2->boundaries)
            {
                if(s1.p1 == s2.p2 && s1.p2 == s2.p1)
                {
                    if(!(s1.p2 == s2.p1))
                    {
                        cout<<"BAD"<<endl;
                    }
                    //cout<<"found one"<<endl;
                    int index1 = distance(n1->boundaries.begin(),find(n1->boundaries.begin(), n1->boundaries.end(), s1));
                    n1->adjacents[index1] = n2;
                    int index2 = distance(n2->boundaries.begin(),find(n2->boundaries.begin(), n2->boundaries.end(), s2));
                    n2->adjacents[index2] = n1;
                }
            }
        }
    }
}

