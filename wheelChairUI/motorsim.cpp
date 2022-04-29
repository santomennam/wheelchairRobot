#include "motorsim.h"

using namespace mssm;

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


constexpr double backEmf(double radiansPerSecond, double voltsPerKRPM)
{
    return (voltsPerKRPM/1000.0)*radiansPerSecToRPM(radiansPerSecond);
}


double MotorSim::calcCurrent()
{
    return (inputVolts - backEmf()) / resistance;
}

double MotorSim::torque()
{
    double motorT = calcCurrent() * torqueConst;

//    if (angVel > 0) {
//        double fricT = constFricTorque + viscFricTorque * angVel;
//    }
//    else if (angVel < 0) {
//        double Frict = constFricTorque - viscFricTorque * angVel;
//    }

    return motorT;
}

void MotorSim::setVoltage(double v)
{
    inputVolts  = v;
  //  cout << "Volts: " <<v << endl;
}

void MotorSim::update(double elapsedSec)
{
    current = calcCurrent();
    if (fabs(current) < 0.001) {
        current = 0;
    }
    double t = torque();
    double a = t / rotationalInertia;
    angVel += a * elapsedSec;
    if (fabs(angVel) < 0.001) {
        angVel = 0;
    }
    angleChange = angVel * elapsedSec;
    angle += angleChange;
}

void MotorSim::draw(Graphics& g, Vec2d pos, double radius)
{
    pos = {pos.x, g.height()-pos.y};
    double diam = radius*2;
    g.ellipse(pos, diam, diam, GREEN);
    g.line(pos, pos + Vec2d{diam/2, 0}.rotated(-wheelAngle()));
}
