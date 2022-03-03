#include "absolutepostracker.h"

AbsolutePosTracker::AbsolutePosTracker()
{
}

bool AbsolutePosTracker::update(int leftEnc, int rightEnc)
{
    bool moved{false};

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
        Dy.rotate(angle+dAngle/2); //taking out dividing by 2 at the end

        moved = da !=0 || db != 0;

        angle += dAngle;
        position = position + Dy;  // Dy was multiplied by 4 before???
    }
    else {
        oldRightEnc = rightEnc;
        oldLeftEnc  = leftEnc;
        gotFirstReading = true;
    }

    return moved;
}
