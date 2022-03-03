#ifndef ABSOLUTEPOSTRACKER_H
#define ABSOLUTEPOSTRACKER_H

#include "vec2d.h"

class RobotParams
{
public:
    static constexpr int    countsPerRev{2400};
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

class AbsolutePosTracker
{
    Vec2d  position;
    double angle{0};
    int    oldLeftEnc;
    int    oldRightEnc;
    bool   gotFirstReading{false};
public:
    AbsolutePosTracker();
    bool update(int leftEnc, int rightEnc);
};

#endif // ABSOLUTEPOSTRACKER_H
