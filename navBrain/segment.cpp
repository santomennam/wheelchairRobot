#include "segment.h"
#include "vec2d.h"
using namespace std;
#include <iostream>

//double segment::magnitude()
//{
//    return (p1-p2).magnitude();
//}

void segment::save(ostream& strm)
{
    strm<<(open?"open":"closed")<<" "<<endl;
    strm<<p1.x<<" "<<p1.y<<" "<<p2.x<<" "<<p2.y<<" "<<endl;
}

void segment::load(istream &strm)
{
    string loadOpen;
    strm>>loadOpen;
    cout<<"loadOpen: "<<loadOpen<<endl;
    if(loadOpen == "open")
    {
        open = true;
    }
    else if(loadOpen == "false")
    {
        open = false;
    }
    else{
        open = false;
        cout<<"unknown string in segment load open/closed"<<endl;
    }
    strm>>p1.x>>p1.y>>p2.x>>p2.y;
    cout<<p1.x<<", "<<p1.y<<" "<<p2.x<<", "<<p2.y<<" "<<endl;
}

segment::segment()
{

}

void segment::magChecker(double width)
{
//    open = (magnitude() > width);
}

segment::segment(Vec2d point1, Vec2d point2, double botWidth)
    :p1{point1},p2{point2}
{
    magChecker(botWidth);
}

double segment::distanceToPoint(Vec2d p, Vec2d& closest)
{
    double t = 0;
    return distanceToPoint(p,closest,t);
}

double segment::distanceToPoint(Vec2d p, Vec2d &closest, double &t)
{
    Vec2d v1 = p2 - p1;
    Vec2d v2 = p - p1;
    t = (v1*v2)/v1.magSquared();
    if(t<=0)
    {
        closest = p1;
        return (p-p1).magnitude();
    }
    else if(t>=1)
    {
        closest = p2;
        return (p-p2).magnitude();
    }
    else
    {
        closest = (p1+(t*v1));
        //subdivide if mag is exactly zero
        return (p-closest).magnitude();
    }
}
double crossSeg(segment a, segment b)
{
    return crossProduct(a.p2-a.p1,b.p2-b.p1);
}

bool operator==(segment a, segment b)
{
    return(a.p1 == b.p1 && a.p2 == b.p2);
}
