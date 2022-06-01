#include "robotmotion.h"
#include <cmath>

class RobotParams
{
public:
    static constexpr int    countsPerRev{1200};
    static constexpr double driveWheelSeparation{21}; // distance between drive wheels
    static constexpr double encSeparation{17};        // distance between encoder wheels (might be same as wheelSeparation)
    static constexpr double driveWheelRadius{5};      // drive wheel radius in inches
    static constexpr double encWheelRadius{1.59};     // encoder wheel radius in inches (might be same as wheelRadius)

    static constexpr double driveWheelCirc()  { return M_PI*2.0*driveWheelRadius;  }
    static constexpr double encWheelCirc()  { return M_PI*2.0*encWheelRadius;  }
    static constexpr double encToRadians(int encoderCount)  { return M_PI*2.0*encoderCount/countsPerRev;}
    static constexpr double driveArcLen(int encoderCount)  { return encToRadians(encoderCount)*driveWheelRadius; }
    static constexpr double encWheelArcLen(int encoderCount)  { return encToRadians(encoderCount)*encWheelRadius; }
};

void calcBotAngleArc(double leftRot, double rightRot, double wheelRadius, double wheelSep, double& dangle, double& arclen)
{
    double arcLeft  = wheelRadius*leftRot;
    double arcRight = wheelRadius*rightRot;

    dangle = (arcRight - arcLeft) / wheelSep;  // change in angle of the axle
    arclen = (arcLeft + arcRight) / 2;  // arclength traveled by axle midpoint
}

void calcBotLeftRightRot(double angle, double arclen, double wheelRadius, double wheelSep, double& leftRot, double& rightRot)
{
    double arcLeft  = arclen - angle*wheelSep/2.0;
    double arcRight = arcLeft + angle*wheelSep;
    leftRot = arcLeft / wheelRadius;
    rightRot = arcRight / wheelRadius;
}

int angleToEncoder(double angle, int countsPerRev)
{
    return angle * countsPerRev / (M_PI*2);
}

double encoderToAngle(double encoder, int countsPerRev)
{
    return encoder * (M_PI*2) / countsPerRev;
}


// assume wheel axle is aligned with x axis, so "forward" displacement is positive Y
void calcBotDisplacement(double leftRot, double rightRot, double wheelRadius, double wheelSep, Vec2d& displacement, double& dangle)
{
    double arcMid;

    calcBotAngleArc(leftRot, rightRot, wheelRadius, wheelSep, dangle, arcMid);

    // deltaY = forward, deltaX = left(-)/right(+)

    // deltaY = arcMid * (sin(dangle)/dangle)        goes to arcMid as angle -> 0
    // deltaX = arcMid * ((1-cos(dangle)) / dangle)  goes to 0 as angle -> 0

    // because of the limits mentioned above,
    //     if dangle < 0.001 we will assume deltaY = arcMid and deltaX = 0

    double deltaX;
    double deltaY;

    if (std::abs(dangle) < 0.001) {
        deltaX = 0;
        deltaY = arcMid;
    }
    else {
        deltaX = -arcMid * ((1-cos(dangle)) / dangle);
        deltaY = arcMid * (sin(dangle)/dangle);
        //cout << "Deltax: " << deltaX << endl;
    }

    displacement = { deltaX, deltaY };
}

// displacement uses pos Y as "forward"
void addDisplacement(Vec2d pos, double angle, Vec2d displacement, double angleChange, double scale, Vec2d& newPos, double& newAngle)
{
    displacement.rotate(angle-M_PI/2.0);
    newPos = pos + displacement * scale;
    newAngle = angle + angleChange;
}


void calcMotion(double leftAngleDelta, double rightAngleDelta,
                int& leftEncoderDelta, int& rightEncoderDelta,
                double& botAngle, Vec2d& botPos)
{
    double robotDeltaAngle;
    double robotArcLen;

    calcBotAngleArc(leftAngleDelta,
                    rightAngleDelta,
                    RobotParams::driveWheelRadius,
                    RobotParams::driveWheelSeparation,
                    robotDeltaAngle,
                    robotArcLen);

    double robotDeltaAngle2;
    Vec2d robotDisplacement;

    calcBotDisplacement(leftAngleDelta,
                    rightAngleDelta,
                    RobotParams::driveWheelRadius,
                    RobotParams::driveWheelSeparation,
                    robotDisplacement,
                    robotDeltaAngle2
                    );


    double leftEncoderAngleChange;
    double rightEncoderAngleChange;

    calcBotLeftRightRot(robotDeltaAngle,
                        robotArcLen,
                        RobotParams::encWheelRadius,
                        RobotParams::encSeparation,
                        leftEncoderAngleChange,
                        rightEncoderAngleChange);

    leftEncoderDelta  = angleToEncoder(leftEncoderAngleChange,  RobotParams::countsPerRev);
    rightEncoderDelta = angleToEncoder(rightEncoderAngleChange, RobotParams::countsPerRev);

    Vec2d newBotPos;
    double newBotAngle;

    addDisplacement(botPos, botAngle, robotDisplacement, robotDeltaAngle2, 1, newBotPos, newBotAngle);

    botPos = newBotPos;
    botAngle = newBotAngle;
}
