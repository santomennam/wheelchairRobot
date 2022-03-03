#ifndef PHYSICS_H
#define PHYSICS_H
#include "graphics.h"
#include "vec2d.h"
#include "segment.h"
#include "viewport.h"
class physics
{
public:
    Vec2d position;
    Vec2d velocity;
    double angle;
    double botWidth;
    double acceleration;
    double angularVelocity;
    double leftPower; //units: rpm or torque - rpm for now
    double rightPower;
    const double wheelDiameter;
    //may need frictional coefficient
    const double distanceBetween;
    const double mass;
    double oldLeftEnc;
    double oldRightEnc;
    double maxMph = 200;
    double maxRpm = (60*maxMph*1.467)/(M_PI*wheelDiameter); //rpm
    void setPower(int left, int right);
    double powerConstant = 10; //conversion between power value and rpm (measured)
public:
    physics();
    void processMovement(double elapsedTime); //where am i after x amount of time
    bool turnTo(Vec2d point); //update power
    bool goTo(Vec2d point,mssm::Graphics& g); //call turn to
    void draw(mssm::Graphics& g,Viewport view);
    Vec2d getXandY(double radiusOfK, double speed);
    Vec2d getPowerCurve(double radiusOfK, double speed);
};

#endif // PHYSICS_H
