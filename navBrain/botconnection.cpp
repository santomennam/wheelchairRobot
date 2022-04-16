#include "botconnection.h"


#include <iostream>

using namespace std;
using namespace mssm;

BotConnection::BotConnection(BotComm *botComm)
    : botComm{botComm}
{
    botComm->attach(this);
}

// called in graphics loop
void BotConnection::update()
{

}

void BotConnection::tankSteer(double left, double right)
{
    stringstream ss;
    ss<<"tank " << left << " " << right;
    sendCommand(ss.str(), true);
}

void BotConnection::setOnTargetUpdateHandler(std::function<void (Vec2d)> onTargetUpdate)
{
    this->onTargetUpdate = onTargetUpdate;
}

void BotConnection::setOnEncoderUpdateHandler(std::function<void (Vec2d)> onEncoderUpdate)
{
    this->onEncoderUpdate = onEncoderUpdate;
}

void BotConnection::setOnMotorUpdateHandler(std::function<void (Vec2d)> onMotorUpdate)
{
    this->onMotorUpdate = onMotorUpdate;
}

double BotConnection::elapsedSinceLastSend() const
{
    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - lastCommandTime;
    return elapsed.count();
}

void BotConnection::updateEncoders(Vec2d encoderCounts)
{
    lastReceivedEncoders = encoderCounts;

    if (onEncoderUpdate) {
        onEncoderUpdate(encoderCounts);
    }
}

void BotConnection::updateTarget(Vec2d encoderTarget)
{
    lastReceivedTarget = encoderTarget;

    if (onTargetUpdate) {
        onTargetUpdate(encoderTarget);
    }
}

void BotConnection::updateMotors(Vec2d motorSpeeds)
{
    lastReceivedMotors = motorSpeeds;

    if (onMotorUpdate) {
        onMotorUpdate(motorSpeeds);
    }
}

void BotConnection::sendCommand(string cmd, bool debug)
{
    if (debug) {
        cout << ">>>>> Sending Command to Bot: " << cmd << endl;
    }
    botComm->sendPacket(cmd);
    lastCommandSent = cmd;
    lastCommandTime = std::chrono::steady_clock::now();
}

void BotConnection::resetBot()
{
    sendCommand("reset", true);
}

void BotConnection::ask()
{
    sendCommand("ask", true);
}

void BotConnection::setDebugMode()
{
    sendCommand("debug", true);
}

void BotConnection::keepBotAlive()
{
    sendCommand("ping", false);
}

void BotConnection::sendTarget(Vec2d encoderTarget)
{
    lastSentTarget = encoderTarget;
    stringstream ss;
    ss<<"target "<<encoderTarget.x<<" "<<encoderTarget.y;
    sendCommand(ss.str(), true);
}


void BotConnection::onBotCommPacket(std::string recCmd)
{
    stringstream dataStream(recCmd);

    char cmd;

    dataStream >> cmd;

    if (recCmd[0] != 'H') {
        receivedCommand = recCmd;  // don't bother recording heartbeat
        cout << "<<<<<< " << receivedCommand << endl;
    }

    double a;
    double b;

    double leftMotor;
    double rightMotor;

    switch (cmd) {
    case 'B':
        receivedError = receivedCommand;
        break;
    case 'R': // reset ack
        receivedError = "";
        receivedInfo = "";
        break;
    case 'D': // debug ack
        break;
    case 'T': // target ack
        dataStream >> a >> b;
        updateTarget({a,b});
        break;
    case 'P': // position
        dataStream >> a >> b >> leftMotor >> rightMotor;
        updateEncoders({a,b});
        updateMotors({leftMotor, rightMotor});
        break;
    case 'E': // Error
        receivedError = receivedCommand;
        break;
    case 'I': // Info
        receivedInfo = receivedCommand;
        break;
    case 'X': // EStop
        break;
    case 'H': // Heartbeat
        break;
    default:
        cout << "UNKNOWN RESPONSE: " << receivedCommand << endl;
        break;
    }
}

void BotConnection::onBotCommConnect()
{
}

void BotConnection::onBotCommDisconnect()
{
}

void BotConnection::onBotCommError(std::string /*msg*/)
{
}
