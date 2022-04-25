#include <iostream>
#include "graphics.h"
#include <iomanip>
#include "absolutepostracker.h"
#include "serialport.h"
#include "CmdLink.h"

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

/*
Torque Constant (Nm/A) 0.05918
Voltage Constant (V/KRPM) 6.2
Motor Inertia (Armature) (kg.mm²) 1386.1
Motor Winding Resistance (Ohms) 0.16
Motor Winding Inductance (μH) 79
Motor Max Winding Temp (ºC) 155
Motor Poles 4
Motor Mass (kg) 6.26
Gear Ratio 25:1
*/

// 1 radian/sec = 9.5493 RPM

//    V/RPM (6.2/1000.0)    1rpm = 2*M_PI/60 radians/sec to Radians/Sec

// 1 rpm == 0.10472 radians/sec

constexpr double radiansPerSecToRPM(double rps) {
    return rps * 9.5493;
}

constexpr double RPMtoRadiansPerSec(double rpm) {
    return rpm * 0.10472;
}

constexpr double backEmf(double radiansPerSecond, double voltsPerKRPM)
{
    return (voltsPerKRPM/1000.0)*radiansPerSecToRPM(radiansPerSecond);
}

class MotorSim {
public:
    double angle{0};       // radians
    double angVel{0};      // radians/second
    double inputVolts{0};  // volts
    double current;        // amps

    double gearRatio{100};

    int    countsPerRev{1200};

    double torqueConst{0.05918};  // Nm/A
    double backEmfConst{6.2};     // voltsPerKRPM
    double resistance{0.16};
    double rotationalInertia{1386.1 / (1000*1000)}; // kg*m^2
    double constFricTorque{1};
    double viscFricTorque{0};

public:
    void   setVoltage(double voltage);
    void   update(double elapsedSec);
    int    encoder();
    void   draw(Graphics& g, Vec2d pos);
    double wheelAngle() { return angle / gearRatio; }
    double wheelRPM()   { return radiansPerSecToRPM(angVel / gearRatio); }
public:
    double backEmf() { return radiansPerSecToRPM(angVel)*backEmfConst/1000.0; }
    double calcCurrent();
    double getCurrent() { return current; }
    double torque();

};

double MotorSim::calcCurrent()
{
    return (inputVolts - backEmf()) / resistance;
}

double MotorSim::torque()
{
    double motorT = calcCurrent() * torqueConst;

    if (angVel > 0) {
        double fricT = constFricTorque + viscFricTorque * angVel;
    }
    else if (angVel < 0) {
        double Frict = constFricTorque - viscFricTorque * angVel;
    }

    return motorT;
}

void MotorSim::setVoltage(double v)
{
    inputVolts  = v;
    cout << "Volts: " <<v << endl;
}

void MotorSim::update(double elapsedSec)
{
    current = calcCurrent();
    double t = torque();
    double a = t / rotationalInertia;
    angVel += a * elapsedSec;
    angle += angVel * elapsedSec;
}



int MotorSim::encoder()
{
    return angle * countsPerRev / (2.0*M_PI);
}

void MotorSim::draw(Graphics& g, Vec2d pos)
{
    pos = {pos.x, g.height()-pos.y};
    double diam = 100;
    g.ellipse(pos, diam, diam, GREEN);
    g.line(pos, pos + Vec2d{diam/2, 0}.rotated(-wheelAngle()));
}

AbsolutePosTracker tracker;

MotorSim m1;
MotorSim m2;

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

    if (abs(dangle) < 0.001) {
        deltaX = 0;
        deltaY = arcMid;
    }
    else {
        deltaX = -arcMid * ((1-cos(dangle)) / dangle);
        deltaY = arcMid * (sin(dangle)/dangle);
    }

    displacement = { deltaX, deltaY };
}

// displacement uses pos Y as "forward"
void addDisplacement(Vec2d pos, double angle, Vec2d displacement, double angleChange, Vec2d& newPos, double& newAngle)
{
    displacement.rotate(angle-M_PI/2.0);
    newPos = pos + displacement;
    newAngle = angle + angleChange;
}


int main()
{
    Graphics g("MyProgram", 1024, 768);

    double angle;
    Vec2d  displacement;

    calcBotDisplacement(3.141, 0, 5, 20, displacement, angle);
    cout << displacement.x << " " << displacement.y << " " << angle << endl;


    calcBotDisplacement(0, 3.141, 5, 20, displacement, angle);
    cout << displacement.x << " " << displacement.y << " " << angle << endl;

    calcBotDisplacement(3.141, 3.141, 5, 20, displacement, angle);
    cout << displacement.x << " " << displacement.y << " " << angle << endl;

    calcBotDisplacement(-3.141, 3.141, 5, 20, displacement, angle);
    cout << displacement.x << " " << displacement.y << " " << angle << endl;

    calcBotDisplacement(3.141, -3.141, 5, 20, displacement, angle);
    cout << displacement.x << " " << displacement.y << " " << angle << endl;

    calcBotDisplacement(20*3.1415/5, 0, 5, 20, displacement, angle);
    cout << displacement.x << " " << displacement.y << " " << angle << endl;

    calcBotDisplacement(0, 20*3.1415/5, 5, 20, displacement, angle);
    cout << displacement.x << " " << displacement.y << " " << angle << endl;


    double arcLen;
    calcBotAngleArc(1,0,5,20,angle,arcLen);
    cout << "A: " << angle << " " << arcLen << endl;
    double leftRot;
    double rightRot;
    calcBotLeftRightRot(angle, arcLen, 5, 20, leftRot, rightRot);
    cout << "Left: " << leftRot << "  Right: " << rightRot << endl;


    calcBotAngleArc(1,1,5,20,angle,arcLen);
    cout << "A: " << angle << " " << arcLen << endl;
    calcBotLeftRightRot(angle, arcLen, 5, 20, leftRot, rightRot);
    cout << "Left: " << leftRot << "  Right: " << rightRot << endl;

    calcBotAngleArc(1,-1,5,20,angle,arcLen);
    cout << "A: " << angle << " " << arcLen << endl;
    calcBotLeftRightRot(angle, arcLen, 5, 20, leftRot, rightRot);

    cout << "Left: " << leftRot << "  Right: " << rightRot << endl;

    m1.setVoltage(0);
    m2.setVoltage(0);

    SerialPort port;

    CmdLink hindbrain(
                [&port](const char* data, int len) { port.write(data, len); },
    [&port]() { return port.canRead(); },
    [&port]() { return port.readChar(); }
    );

    hindbrain.setDebug(true);

    bool portOpen = port.open("COM6", 115200);

    if (!portOpen) {
        g.cerr << "Could not open serial port" << endl;
    }

    if (portOpen) {
        hindbrain.sendCmd('U'); // turn on emUlate mode
    }

    int lastLeftEnc = m1.encoder();
    int lastRightEnc = m2.encoder();

    while (g.draw()) {

        tracker.update(m1.encoder(), m2.encoder());

        int leftEnc = m1.encoder();
        int rightEnc = m2.encoder();

        if (portOpen) {
            if (lastLeftEnc != leftEnc || lastRightEnc != rightEnc) {
                cout << "Sending: " << leftEnc << " " << rightEnc << endl;
                hindbrain.sendCmdII('C',leftEnc,rightEnc);
                lastLeftEnc = leftEnc;
                lastRightEnc = rightEnc;
                // g.cerr << "Sent encoders" << endl;
            }

            if (hindbrain.readCmd()) {
                char m1power;
                char m2power;

                switch (hindbrain.cmd()) {
                case 'M': // motor value
                    hindbrain.getParam(m1power);
                    hindbrain.getParam(m2power);
                    m1.setVoltage(m1power/4);
                    m2.setVoltage(m2power/4);
                }
            }
        }

        m1.update(g.elapsedMs()/1000);
        m2.update(g.elapsedMs()/1000);

        m1.draw(g, {150, 150});
        m2.draw(g, {350, 150});

        g.cout << "V        " << m1.inputVolts << endl;
        g.cout << "I        " << m1.getCurrent() << endl;
        g.cout << "BackEmf  " << m1.backEmf() << endl;
        g.cout << "M1 Rpm:  " << m1.wheelRPM() << endl;
        g.cout << "M2 Rpm:  " << m2.wheelRPM() << endl;

        g.cout << "Angle: " << tracker.getAngle() << endl;
        g.cout << "X: " << tracker.getPos().x << endl;
        g.cout << "Y: " << tracker.getPos().y << endl;
        g.cout << "E1: " << m1.encoder() << endl;
        g.cout << "E2: " << m2.encoder() << endl;

        Vec2d botPos = tracker.getPos();

        botPos = { botPos.x, g.height()-botPos.y };

        g.ellipse(botPos, 20, 20, GREEN);
        g.line(botPos, botPos + Vec2d{20, 0}.rotated(-tracker.getAngle()), GREEN);

        if (g.isKeyPressed(Key::ESC)) {
            break;
        }

//        if (g.isKeyPressed('Q')) {
//            m1.setVoltage(10);
//        }
//        else if (g.isKeyPressed('A')) {
//            m1.setVoltage(5);
//        }
//        else if (g.isKeyPressed('Z')) {
//            m1.setVoltage(-5);
//        }
//        else {
//            m1.setVoltage(0);
//        }

//        if (g.isKeyPressed('U')) {
//            m2.setVoltage(10);
//        }
//        else if (g.isKeyPressed('J')) {
//            m2.setVoltage(5);
//        }
//        else if (g.isKeyPressed('M')) {
//            m2.setVoltage(-5);
//        }
//        else {
//            m2.setVoltage(0);
//        }

        for (const Event& e : g.events()) {
            switch (e.evtType) {
            case EvtType::KeyPress:
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


