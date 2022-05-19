#include "waypoint.h"

//using namespace std;
//Waypoint::Waypoint(cell *c, Waypoint *prev, Vec2d destination)
//{
//    previous = prev;
//    this->c = c;
//    c->color = mssm::YELLOW;
//    Vec2d garbage;
//    hueristic = c->distanceToPoint(destination, garbage);
//    if(prev){
//        cost = prev->cost + (n->centroid()-prev->node->centroid()).magnitude();
//    }
//    else{
//        cost = 0;
//    }

//}

//bool Waypoint::containsNode(Node *n)
//{
//    if(node == n)
//    {
//        return true;
//    }
//    if(previous)
//    {
//        return previous->containsNode(n);
//    }
//    return false;
//}

//std::vector<Node *> Waypoint::extract()
//{
//    vector<Node*> path;
//    if(previous){
//        path = previous->extract();
//    }
//    path.push_back(node);
//    return path;
//}

//std::vector<Vec2d> Waypoint::pathPoints(double botWidth)
//{
//    vector<Vec2d> pointsReturn;
//    vector<Node*> path = extract();

//    //get nodes out of this order and into a list
//    for(int i =0; i<path.size(); i++)
//    {
//        pointsReturn.push_back(path[i]->centroid()); //this lines are a temp fix

//        //        Vec2d garbage;
//        //        auto n = path[i];
//        //        bool centroid = true;
//        //        if(pointsReturn.size() && path.size()>i+1)
//        //        {
//        //            segment* useThis = nullptr;

//        //            vector<segment> prospects =path[i]->sharedSegs(path[i+1]);
//        //            for(int i =0; i <prospects.size(); i++)
//        //            {
//        //                if(prospects[i].open)
//        //                {
//        //                    useThis = &prospects[i];
//        //                    break;
//        //                }
//        //            }
//        //            if(!useThis)
//        //            {
//        //                //cout<< "1 uh oh!!!!! useThis doesnt exist. Path planner is broken."<<endl;
//        //            }
//        //            segment testSeg{pointsReturn.back(),useThis->midpoint(),20};
//        //            //cout<<"1 probably not handling this right bc potentially more than one segment between these two nodes because of splitEdge"<<endl;

//        //            vector<Vec2d> bounds = path[i]->boundaryPoints();
//        //            centroid = false;
//        //            for(int i = 0; i < bounds.size(); i++)
//        //            {
//        //                if(testSeg.distanceToPoint(bounds[i],garbage)<botWidth)
//        //                {
//        //                    centroid = true;
//        //                }
//        //            }

//        //        }

//        //        if(centroid)
//        //        {
//        //            pointsReturn.push_back(n->centroid());
//        //        }
//        //        if(path.size()>i+1)
//        //        {
//        //            segment* useThis = nullptr;
//        //            vector<segment> prospects =path[i]->sharedSegs(path[i+1]);
//        //            for(int i =0; i <prospects.size(); i++)
//        //            {
//        //                if(prospects[i].open)
//        //                {
//        //                    useThis = &prospects[i];
//        //                    break;
//        //                }
//        //            }
//        //            if(!useThis)
//        //            {
//        //                cout<< "2 uh oh!!!!! useThis doesnt exist. Path planner is broken."<<endl;
//        //            }
//        //            pointsReturn.push_back(useThis->midpoint());
//        //            //cout<<"2 probably not handling this right bc potentially more than one segment between these two nodes because of splitEdge"<<endl;

//        //        }
//    }
//    return pointsReturn;
//}

//void Waypoint::recalculateCost(Vec2d destination)
//{
//    Vec2d garbage;
//    hueristic = node->distanceToPoint(destination, garbage);
//    if(previous){
//        cost = previous->cost + (node->centroid()-previous->node->centroid()).magnitude();
//    }
//    else{
//        cost = 0;
//    }
//}
//bool operator<(const Waypoint& w1, const Waypoint& w2)
//{
//    return((w1.hueristic+w1.cost)<(w2.hueristic+w2.cost));
//}
