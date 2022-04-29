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

// Note:  Mac Users who are using Qt Creator...
//        if the program compiles and runs but a window doesn't open up
//        try un-checking "run in terminal"


// Here are some of the basic drawing commands:

//void   line(Vec2d p1, Vec2d p2, Color c = WHITE);
//void   ellipse(Vec2d center, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
//void   arc(Vec2d center, double w, double h, double a, double alen, Color c = WHITE);
//void   chord(Vec2d center, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
//void   pie(Vec2d center, double w, double h, double a, double alen, Color c = WHITE, Color f = TRANSPARENT);
//void   rect(Vec2d corner, double w, double h, Color c = WHITE, Color f = TRANSPARENT);
//void   polygon(std::vector<Vec2d> pts, Color border, Color fill = TRANSPARENT);
//void   polyline(std::vector<Vec2d> pts, Color color);
//void   text(Vec2d pos, double size, const std::string& str, Color textColor = WHITE, HAlign hAlign = HAlign::left, VAlign vAlign = VAlign::baseline);

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

enum class BotState
{
  asleep,   // motors off, brakes engaged
  waking,   // motors off, brakes releasing       transition to awake after brake timer expires
  awake,    // brakes off, motors may be running
  stopping, // motors off, brakes engaging        transition to asleep after brake timer expires
  estop,    // estop was triggered, motors off, brakes engaged, can only be brought out of this mode by a reset command
};

int main()
{
    Graphics g("Bot Emulator", 1024, 768);

    Vec2d botPos{0, 0};
    double botAngle{0};

    BotState botState{BotState::asleep};

    int leftEncoderCount{0};
    int rightEncoderCount{0};

    Vec2d startPos = botPos;

    m1.setVoltage(0);
    m2.setVoltage(0);

    SerialPort port;

    CmdLink hindbrain(
                [&port](const char* data, int len) { port.write(data, len); },
    [&port]() { return port.canRead(); },
    [&port]() { return port.readChar(); }
    );

    //hindbrain.setDebug(true);

    bool portOpen = port.open("COM6", 115200);

    if (!portOpen) {
        g.cerr << "Could not open serial port" << endl;
    }

    if (portOpen) {
        hindbrain.sendCmd('U'); // turn on emUlate mode
    }

    int lastLeftEnc = -1;
    int lastRightEnc = -1;

    while (g.draw()) {

        string state;
        Color stateColor;

        switch (botState) {
          case BotState::asleep:
            state = "Asleep";
            stateColor = BLUE;
            break;
          case BotState::waking:
            state = "Waking";
            stateColor = CYAN;
            break;
          case BotState::awake:
            state = "Awake";
            stateColor = GREEN;
            break;
          case BotState::stopping:
            state = "Stopping";
            stateColor = YELLOW;
            break;
          case BotState::estop:
            state = "EStop";
            stateColor = RED;
            break;
        }

        g.text({250, 30}, 20, "State: " + state, stateColor);


        if (portOpen) {

            while (hindbrain.readCmd()) {
                char m1power;
                char m2power;

                switch (hindbrain.cmd()) {
                case 'M': // motor value
                    hindbrain.getParam(m1power);
                    hindbrain.getParam(m2power);
                    m1.setVoltage(m1power/4);
                    m2.setVoltage(m2power/4);
                    break;
                case 'I':
                    g.cerr << hindbrain.getStr();
                    break;
                case 'w':
                    botState = BotState::waking;
                    break;
                case 'W':
                    botState = BotState::awake;
                    break;
                case 's':
                    botState = BotState::stopping;
                    break;
                case 'S':
                    botState = BotState::asleep;
                    break;
                case 'X':
                    botState = BotState::estop;
                    break;

                default:
                    cout << "Some other msg from hindbrain: " << hindbrain.cmd() << endl;
                    break;
                }
            }
        }

        m1.update(g.elapsedMs()/1000);
        m2.update(g.elapsedMs()/1000);

        int leftEncoderDelta;
        int rightEncoderDelta;

        calcMotion(m1.wheelAngleChange(), m2.wheelAngleChange(),
                   leftEncoderDelta, rightEncoderDelta,
                   botAngle, botPos);

//        double robotDeltaAngle;
//        double robotArcLen;

//        calcBotAngleArc(m1.wheelAngleChange(),
//                        m2.wheelAngleChange(),
//                        RobotParams::driveWheelRadius,
//                        RobotParams::driveWheelSeparation,
//                        robotDeltaAngle,
//                        robotArcLen);

//        double robotDeltaAngle2;
//        Vec2d robotDisplacement;

//        calcBotDisplacement(m1.wheelAngleChange(),
//                        m2.wheelAngleChange(),
//                        RobotParams::driveWheelRadius,
//                        RobotParams::driveWheelSeparation,
//                        robotDisplacement,
//                        robotDeltaAngle2
//                        );


//        double leftEncoderAngleChange;
//        double rightEncoderAngleChange;

//        calcBotLeftRightRot(robotDeltaAngle,
//                            robotArcLen,
//                            RobotParams::encWheelRadius,
//                            RobotParams::encSeparation,
//                            leftEncoderAngleChange,
//                            rightEncoderAngleChange);

//        int leftEncoderDelta  = angleToEncoder(leftEncoderAngleChange,  RobotParams::countsPerRev);
//        int rightEncoderDelta = angleToEncoder(rightEncoderAngleChange, RobotParams::countsPerRev);

        leftEncoderCount += leftEncoderDelta;
        rightEncoderCount += rightEncoderDelta;

        tracker.update(leftEncoderCount, rightEncoderCount);

        if (portOpen) {
            if (lastLeftEnc != leftEncoderCount || lastRightEnc != rightEncoderCount) {
                cout << "Sending: " << leftEncoderCount << " " << rightEncoderCount << endl;
                hindbrain.sendCmdII('C', leftEncoderCount, rightEncoderCount);
                lastLeftEnc  = leftEncoderCount;
                lastRightEnc = rightEncoderCount;
                // g.cerr << "Sent encoders" << endl;
            }
        }


//        Vec2d newBotPos;
//        double newBotAngle;

//        addDisplacement(botPos, botAngle, robotDisplacement, robotDeltaAngle2, 1, newBotPos, newBotAngle);

//        botPos = newBotPos;
//        botAngle = newBotAngle;


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

        if (!portOpen) {
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

        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
                switch (e.arg) {
                case 'E':
                    hindbrain.sendCmd('U');
                    break;
                case 'L':
                    hindbrain.sendCmd('L');
                    break;
                case 'W':
                    hindbrain.sendCmd('W');
                    break;
                case 'C':
                    hindbrain.sendCmdII('C',1234567,8912345);
                    break;
                case 'F':
                    hindbrain.sendCmdII('C',++fakeEncoderL,++fakeEncoderR);
                    break;
                }

                break;
            case EvtType::KeyRelease:
                break;
            case EvtType::MouseMove:
                break;
            case EvtType::MousePress:
                break;
            case EvtType::MouseRelease:
                break;
            default:
                break;
            }
        }
    }

    return 0;
}


