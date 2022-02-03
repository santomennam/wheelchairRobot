#include "physics.h"
#include <iostream>
#include "graphics.h"
using namespace std;
using namespace mssm;
physics::physics()
    :wheelDiameter{10}, distanceBetween{21},mass{100},leftPower{0},rightPower{0} //mass is a guess, other two are in inches
{
}

void physics::processMovement(double elapsedTime) //milliseconds
{
    if(elapsedTime<1)
    {
        cout<<"time too small!!!"<<endl;
        throw new logic_error("time too small in processMovement!");
    }
    double timeScale =1;
    int loop = elapsedTime/timeScale;
    double stepTime = elapsedTime/loop;
    for(int i = 0; i <loop; i++)
    {
        //   cout<<"process"<<endl;
        double cpr = 600; //counts per revolution
        //  double anglePrev = angle;
        double rightEnc = 0;
        double leftEnc = 0;
        //elapsed time * rpm calcs distance
        double da = leftPower * stepTime/60000 *cpr; //convert from rpm to r/millisecond
        double db = rightPower * stepTime/60000 *cpr;
        leftEnc = oldLeftEnc + da;
        rightEnc = oldRightEnc +db;

        //  dataStream>>robot.distanceRead>>a>>b; // uwu HERE
        //    robot.distanceRead /= 2.54;
        //    robot.distanceRead += 8;
        leftEnc=leftEnc/4;
        rightEnc=rightEnc/4;
        //        storeLeft += da;
        //        storeRight += db;
        //  cout<<"Left: "<<leftEnc<<" Right: "<<rightEnc<<" da: "<<da<<" db: "<<endl;

        if(leftEnc != oldLeftEnc ||rightEnc  != oldRightEnc)
        {
            // cout<<" a: "<<a<<" b "<<b<<endl;
        }

        oldLeftEnc = leftEnc;
        oldRightEnc = rightEnc;

        double aDist = da*((2*M_PI)*(1.59375)/cpr); //gonna cry?
        double bDist = db*((2*M_PI)*(1.59375)/cpr);
        //  double dx =0; // r*(cos((bDist/r))-cos((aDist/r)));
        angularVelocity = (bDist-aDist)/(distanceBetween); //might be double this (half dist between?)
        double dy = -(aDist/2+bDist/2);
        Vec2d Dy{dy,0};
        Dy.rotate(angle+(angularVelocity)/2);
        if(da !=0 ||db != 0)
        {
            //robot.moved = true;
            // cout<<"aDist: "<<aDist<<" da: "<<da<<" db: "<<db<<" bDist: "<<bDist<<" dy: "<<dy<<" d angle: "<<((angle)/M_PI)*180<<" robot angle: "<<((angle)/M_PI)*180<<endl;
            // cout<<position.x<<", "<<position.y<<endl;
        }
        else{
            //robot.moved = false;
        }

        angle += angularVelocity;
        //  cout<<angularVelocity<<endl;
        velocity = (Dy); //*4?
        position = position + velocity;
    }
}
double angleBetween(Vec2d p1, Vec2d p2)
{
    return atan2(crossProduct(p1,p2),p1*p2);
}

bool physics::turnTo(Vec2d point)
{
    //THIS IS IN RADIANS
    double epsilon = 0.05;
    //find where we want to be pointing to
    if(point == position)
    {
        return true;
    }
    Vec2d dir1 = Vec2d{1,0}.rotated(angle);
    Vec2d dir2 = (point-position).unit();
    double dangle = angleBetween(dir1,dir2);
    // cout<<dangle<<endl;
    //set power so we will turn towards angle, stop when we are at angle - we shouldnt loop inside here: run this a million times
    if(abs(dangle)<epsilon)
    {
        setPower(0,0);
        return true;
    }
    else{
        setPower(-100*dangle,100*dangle);
    }

    return false;

    //need normalize .unit
    //normalize
    //turn
}
void physics::setPower(int left, int right)
{
    int maxPower = 2000;
    rightPower = max(min(left,maxPower),-maxPower);
    leftPower = max(min(right,maxPower),-maxPower);
}
Vec2d physics::getXandY(double radiusOfK, double speed)
{
    //x and y are the left and right arc lengths of the robot's motion. calculated from power ig
    double a = ((3*botWidth)/(2*radiusOfK))+1;
    double y = 2*speed/(a+1);
    double x = a*y;
    return{x,y};
}

Vec2d physics::getPowerCurve(double radiusOfK, double speed)
{//time is in seconds, speed in rpm
    double maxPower = 250;
    Vec2d values = getXandY(radiusOfK,speed);
    double leftRotations = values.x/(wheelDiameter*M_2_PI);
    double rightRotations = values.y/(wheelDiameter*M_2_PI);
    Vec2d powers = {(leftRotations) * powerConstant, (rightRotations) * powerConstant}; //power constant turns rpm (l/r rotations over time) to powers. need to run the motors for the time passed in in order to do the curve.
    double maxAbs = max(abs(powers.x),abs(powers.y));
    if(maxAbs>maxPower)
    {
        double scale = maxPower/maxAbs;
        powers = powers*scale;
    }
    return powers;
}

bool physics::goTo(Vec2d point, mssm::Graphics& g)
{
    double dist = (position-point).magnitude();
    if(dist < 1){
        setPower(0,0);
        return true;
    }
    Vec2d dir1 = Vec2d{1,0}.rotated(angle);
    Vec2d dir2 = (point-position).unit();
    double dangle = angleBetween(dir1,dir2);
    double maxPower = 40;
    if(abs(dangle) < 0.02) //angle of some kind
    {
        setPower(20,20);
    }
    else{
      //  cout<<"in the super special section"<<endl;
        double radius = max(min((((600/M_PI)*dangle)+600),600.0),1.0) * dangle/abs(dangle); //angle over abs dangle makes sure the sign is correct //inches
        g.text(200,600,20,to_string(radius));
        g.text(200,650,20,to_string(dangle));

        leftPower = getPowerCurve(radius,20).x;
        rightPower = getPowerCurve(radius,20).y;
    }
//    //cout<<"power in goTo: left: "<<leftPower<<" right: "<<rightPower<<endl;
//    if(turnTo(point)){
//        double dist = (position-point).magnitude();
//        if(dist>0.05){
//            setPower(-10*dist,-10*dist);
//            return false;
//        }
//        else{
//            setPower(0,0);
//            return true;
//        }
//    }
    return false;
}

void physics::draw(Graphics& g, Viewport view)
{
    g.line(view.worldToScreen(position),view.worldToScreen(position+Vec2d{50,0}.rotated(angle)),WHITE);

    g.ellipseC(view.worldToScreen(position),20*view.scale,20*view.scale,RED,RED);
}
