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

double BotConnection::elapsedSinceLastResponse() const
{
    std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - lastResponseTime;
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
    if (isWaitingForResponse) {
        cout << ">>>>>>>>>>>>>>> sendCommand didn't wait for response <<<<<<<<<<<<<" << endl;
    }
    if (debug) {
        cout << ">>>>> Sending Command to Bot: " << cmd << endl;
    }
    botComm->sendPacket(cmd);
    lastCommandSent = cmd;
    lastCommandTime = std::chrono::steady_clock::now();
    isWaitingForResponse = true;
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
    //cout << recCmd << endl;
    stringstream dataStream(recCmd);

    char cmd;

    dataStream >> cmd;

    double a;
    double b;

    double leftMotor;
    double rightMotor;

    switch (cmd) {
    case 'A':
        // Ack: Acknowledge
        lastResponseTime = std::chrono::steady_clock::now();
        isWaitingForResponse = false;
        break;
    case 'N':
        // Nack: Negative Acknowledge
        lastResponseTime = std::chrono::steady_clock::now();
        isWaitingForResponse = false;
        cout << "NACKNACKNACK" << endl;
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
        receivedError = receivedResponse;
        break;
    case 'I': // Info
        receivedInfo = receivedResponse;
        break;
    case 'X': // EStop
        break;
    default:
        cout << "UNKNOWN RESPONSE: " << receivedResponse << endl;
        break;
    }
}

void BotConnection::onBotCommConnect()
{
}

void BotConnection::onBotCommDisconnect()
{
}

void BotConnection::onBotCommError(std::string msg)
{
    receivedError = msg;
}
