#include "absolutepostracker.h"


AbsolutePosTracker::AbsolutePosTracker()
{
    start = std::chrono::steady_clock::now();
    position = {0,0};
}

bool AbsolutePosTracker::update(int leftEnc, int rightEnc)
{
    bool moved{false};
    double left = leftEnc;
    double right = rightEnc;
    encoderReadings = {left,right};

    if(gotFirstReading) {
        double db = rightEnc-oldRightEnc;
        double da = leftEnc-oldLeftEnc;

        oldRightEnc = rightEnc;
        oldLeftEnc = leftEnc;

        double leftDist = RobotParams::encWheelArcLen(da);
        double rightDist = RobotParams::encWheelArcLen(db);

        double dAngle = (rightDist-leftDist) / RobotParams::encSeparation;
        double dy    = -(leftDist/2+rightDist/2);
        Vec2d Dy{dy,0};
        Dy.rotate(angle+dAngle/2); //taking out dividing by 2 at the end //<-- what the fuck does this mean

        moved = da !=0 || db != 0;


        angle += dAngle;
        position = position - Dy; // changed from + to - 3/31/22

        if(moved){
            navPoints.push_back(navPoint(position,angle,encoderReadings));
        }

    }
    else {
        oldRightEnc = rightEnc;
        oldLeftEnc  = leftEnc;
        gotFirstReading = true;
    }

    return moved;
}

navPoint::navPoint(Vec2d pos, double angle, Vec2d encoderCounts, bool turn)
{
    encoderReadings = encoderCounts;
    this->turn = turn;
    time = std::chrono::steady_clock::now();
    this->pos = pos;
    this->angle = angle;
}
