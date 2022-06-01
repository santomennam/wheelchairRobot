#ifndef MOTORSIM_H
#define MOTORSIM_H

#include "graphics.h"

constexpr inline double radiansPerSecToRPM(double rps) {
    return rps * 9.5493;
}

constexpr inline double RPMtoRadiansPerSec(double rpm) {
    return rpm * 0.10472;
}


class MotorSim {
public:
    double angle{0};       // radians
    double angleChange{0}; // amount angle changed this update
    double angVel{0};      // radians/second
    double inputVolts{0};  // volts
    double current;        // amps

    double gearRatio{25};

    double torqueConst{0.05918};  // Nm/A
    double backEmfConst{6.2};     // voltsPerKRPM
    double resistance{0.16};
    double rotationalInertia{1386.1 / (1000*1000)}; // kg*m^2
    double constFricTorque{1};
    double viscFricTorque{0};

public:
    void   setVoltage(double voltage);
    void   update(double elapsedSec);
    void   draw(mssm::Graphics& g, Vec2d pos, double radius);
    double wheelAngle() { return angle / gearRatio; }
    double wheelAngleChange() { return angleChange / gearRatio; }
    double wheelRPM()   { return radiansPerSecToRPM(angVel / gearRatio); }
public:
    double backEmf() { return radiansPerSecToRPM(angVel)*backEmfConst/1000.0; }
    double calcCurrent();
    double getCurrent() { return current; }
    double torque();

};

#endif // MOTORSIM_H
