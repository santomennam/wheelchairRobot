#include <iostream>
#include "graphics.h"
#include <iomanip>
#include "absolutepostracker.h"
#include "serialport.h"
#include "CmdLink.h"
#include "robotmotion.h"
#include "motorsim.h"

using namespace std;
using namespace mssm;

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

AbsolutePosTracker tracker;

MotorSim m1;
MotorSim m2;

int fakeEncoderL = 0;
int fakeEncoderR = 0;

void drawBot(Graphics& g, Vec2d botPos, double botAngle, double scale, Color color)
{
    Vec2d bp = Vec2d{g.width()/2, g.height()/2} + Vec2d{botPos.x, -botPos.y} * scale;
    g.ellipse(bp, 20*scale, 20*scale, color);
    g.line(bp, bp + Vec2d{11*scale, 0}.rotated(-botAngle), color);
}

int main()
{
    Graphics g("Bot Emulator", 1024, 768);

    Vec2d botPos{0, 0};
    double botAngle{0};

    int leftEncoderCount{0};
    int rightEncoderCount{0};

    Vec2d startPos = botPos;

    m1.setVoltage(0);
    m2.setVoltage(0);

    int lastLeftEnc = -1;
    int lastRightEnc = -1;

    while (g.draw()) {

        m1.update(g.elapsedMs()/1000);
        m2.update(g.elapsedMs()/1000);

        int leftEncoderDelta;
        int rightEncoderDelta;

        calcMotion(m1.wheelAngleChange(), m2.wheelAngleChange(),
                   leftEncoderDelta, rightEncoderDelta,
                   botAngle, botPos);

        leftEncoderCount += leftEncoderDelta;
        rightEncoderCount += rightEncoderDelta;

        tracker.update(leftEncoderCount, rightEncoderCount);

        double scale = 5;

        m1.draw(g, {150, 150}, scale * RobotParams::driveWheelRadius);
        m2.draw(g, {350, 150}, scale * RobotParams::driveWheelRadius);

        g.cout << "V        " << m1.inputVolts << endl;
        g.cout << "I        " << m1.getCurrent() << endl;
        g.cout << "BackEmf  " << m1.backEmf() << endl;
        g.cout << "M1 Rpm:  " << m1.wheelRPM() << endl;
        g.cout << "M2 Rpm:  " << m2.wheelRPM() << endl;

        g.cout << "Angle: " << botAngle << endl;
        g.cout << "X: " << botPos.x << endl;
        g.cout << "Y: " << botPos.y << endl;

        g.cout << "LeftEnc:  " << leftEncoderCount << endl;
        g.cout << "RightEnc: " << rightEncoderCount << endl;

        drawBot(g, botPos, botAngle, scale, GREEN);
        drawBot(g, tracker.getPos(), tracker.getAngle(), scale, YELLOW);

        if (g.isKeyPressed(Key::ESC)) {

            break;
        }

        if (g.isKeyPressed('Q')) {
            m1.setVoltage(10);
        }
        else if (g.isKeyPressed('A')) {
            m1.setVoltage(5);
        }
        else if (g.isKeyPressed('Z')) {
            m1.setVoltage(-5);
        }
        else {
            m1.setVoltage(0);
        }

        if (g.isKeyPressed('U')) {
            m2.setVoltage(10);
        }
        else if (g.isKeyPressed('J')) {
            m2.setVoltage(5);
        }
        else if (g.isKeyPressed('M')) {
            m2.setVoltage(-5);
        }
        else {
            m2.setVoltage(0);
        }

    }

    return 0;
}


