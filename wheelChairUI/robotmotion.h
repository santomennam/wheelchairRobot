#ifndef ROBOTMOTION_H
#define ROBOTMOTION_H

#include "vec2d.h"

void calcMotion(double leftAngleDelta, double rightAngleDelta,
                int& leftEncoderDelta, int& rightEncoderDelta,
                double& botAngle, Vec2d& botPos);

#endif // ROBOTMOTION_H
