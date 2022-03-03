#include "robot.h"


using namespace std;
using namespace mssm;

Robot::Robot(Vec2d startPosition)
{
    velocity={0,0};
    angle=0;
    speed=0;
    position = startPosition;
}
//void Robot::navigation()
//{

//}

void Robot::measureStepOne() //show/hide map
{
    int i =0;
    vector<Vec2d> points;
    while(points.size()<distanceRead)
    {
        i++;
        Vec2d point;
        point.x = 10*i;
        point.y = 0;
        point.rotate(angle);
        point = position + point;
        points.push_back(point);
    }
    measuredPoints = points;
}

void Robot::update()
{
//    double anglePrev = angle;
//    Vec2d velPrev = velocity;
//    angle += angularVelocity;
//    velocity = Vec2d{speed,0}.rotated(angle);
//    position = velocity + position;

    //moved = (angularVelocity!=0 || velocity.magnitude()!=0); //detect movement

    measureStepOne();
}
