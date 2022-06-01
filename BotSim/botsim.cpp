#include "botsim.h"
#include "robotmotion.h"

BotSim::BotSim()
{
    setMotors(0,0);
}

void BotSim::setMotors(int m1Power, int m2Power)
{
    m1.setVoltage(m1Power);
    m2.setVoltage(m2Power);
}

void BotSim::update(double elapsedSeconds)
{
    m1.update(elapsedSeconds);
    m2.update(elapsedSeconds);

    int leftEncoderDelta;
    int rightEncoderDelta;

    calcMotion(m1.wheelAngleChange(), m2.wheelAngleChange(),
               leftEncoderDelta, rightEncoderDelta,
               angle, position);

    leftEncoder += leftEncoderDelta;
    rightEncoder += rightEncoderDelta;
}
