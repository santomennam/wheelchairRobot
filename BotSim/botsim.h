#ifndef BOTSIM_H
#define BOTSIM_H

#include "motorsim.h"

class BotSim
{
private:
    MotorSim m1;
    MotorSim m2;
    Vec2d  position;
    double angle;
    int leftEncoder{0};
    int rightEncoder{0};
public:
    BotSim();
    void setMotors(int m1Power, int m2Power);
    void update(double elapsedSeconds);
    Vec2d getPos()    { return position;     }
    double getAngle() { return angle;        }
    int getLeftEnc()  { return leftEncoder;  }
    int getRightEnc() { return rightEncoder; }
};

#endif // BOTSIM_H
