#include <iostream>
#include "graphics.h"
#include <iomanip>
#include "absolutepostracker.h"
#include "serialport.h"
#include "CmdLink.h"
#include "robotmotion.h"
#include "motorsim.h"
#include "botsim.h"

using namespace std;
using namespace mssm;

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif



void drawBot(Graphics& g, Vec2d botPos, double botAngle, double scale, Color color)
{
    Vec2d bp = Vec2d{g.width()/2, g.height()/2} + Vec2d{botPos.x, -botPos.y} * scale;
    g.ellipse(bp, 20*scale, 20*scale, color);
    g.line(bp, bp + Vec2d{11*scale, 0}.rotated(-botAngle), color);
}

int main()
{
    Graphics g("Bot Emulator", 1024, 768);

    BotSim bot;

    vector<Vec2d> trail;

    trail.push_back(bot.getPos());

    while (g.draw()) {

        bot.update(g.elapsedMs()/1000);

        double scale = 5;

    //    m1.draw(g, {150, 150}, scale * RobotParams::driveWheelRadius);
    //    m2.draw(g, {350, 150}, scale * RobotParams::driveWheelRadius);

//        g.cout << "V        " << m1.inputVolts << endl;
//        g.cout << "I        " << m1.getCurrent() << endl;
//        g.cout << "BackEmf  " << m1.backEmf() << endl;
//        g.cout << "M1 Rpm:  " << m1.wheelRPM() << endl;
//        g.cout << "M2 Rpm:  " << m2.wheelRPM() << endl;

//        g.cout << "Angle: " << botAngle << endl;
//        g.cout << "X: " << botPos.x << endl;
//        g.cout << "Y: " << botPos.y << endl;

        g.cout << "LeftEnc:  " << bot.getLeftEnc() << endl;
        g.cout << "RightEnc: " << bot.getRightEnc() << endl;

        if ((trail.back() - bot.getPos()).magnitude() > 2) {
            trail.push_back(bot.getPos());
        }

        drawBot(g, bot.getPos(), bot.getAngle(), scale, GREEN);

        vector<Vec2d> trailTmp = trail;

        for (auto& p : trailTmp) {
            p = Vec2d{g.width()/2, g.height()/2} + Vec2d{p.x, -p.y} * scale;

        }

        g.polyline(trailTmp, WHITE);

        if (g.isKeyPressed(Key::ESC)) {

            break;
        }

        double m1Volts = 0;
        double m2Volts = 0;

        if (g.isKeyPressed('Q')) {
            m1Volts = 10;
        }
        else if (g.isKeyPressed('A')) {
            m1Volts = 5;
        }
        else if (g.isKeyPressed('Z')) {
            m1Volts = -5;
        }

        if (g.isKeyPressed('U')) {
            m2Volts = 10;
        }
        else if (g.isKeyPressed('J')) {
            m2Volts = 5;
        }
        else if (g.isKeyPressed('M')) {
            m2Volts = -5;
        }

        bot.setMotors(m1Volts, m2Volts);

    }

    return 0;
}


