#include "obstacle.h"
#include <iostream>
using namespace std;
Obstacle::Obstacle(vector<Vec2d> points)
    :pts{points}
{
    interpolate();
}

void Obstacle::interpolate()
{
    cout<<"placing points"<<endl;
    vector<Vec2d> tempPoints;
    for(int i =0; i<pts.size()-1;i++) //ISSUE
    {
        tempPoints.push_back(pts[i]);
        Vec2d distances(pts[i+1]-pts[i]);
        double numPoints = distances.magnitude()/10; //adjust the divisor to adjust distance between points
        for(int j = 0; j<numPoints; j++)
        {
            double t = j/numPoints;
            Vec2d point((pts[i]+distances*(t)));
            tempPoints.push_back(point);
            cout<<"placed "<<point<<" as a point"<<endl;
            //how do i pushback and maintain order? help
        }
    }
    tempPoints.push_back(pts[pts.size()-1]);
    pts = tempPoints;
}
