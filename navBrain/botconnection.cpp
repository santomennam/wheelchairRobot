#include "botconnection.h"


#include <iostream>

using namespace std;
using namespace mssm;

BotConnection::BotConnection(BotComm *bc)
    : botComm{bc}
{
    botComm->attach(this);
    cmdLink = new CmdLink([this](const char* data, int len) {
        botComm->sendPacket(string(data, len));
    },
    [this]() { return this->incomingData.size() > 0; },
    [this]() { char c = incomingData[0]; incomingData.erase(incomingData.begin()); return c; });
}

// called in graphics loop
void BotConnection::update()
{

}

void BotConnection::tankSteer(double left, double right)
{
    stringstream ss;
    ss<<"tank " << left << " " << right;
    sendCommand(ss.str(), logLevel > 0);
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
    lastCmdPing = cmd == "ping";

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
    sendCommand("reset", logLevel > 0);
}

void BotConnection::ask()
{
    sendCommand("ask", logLevel > 0);
}

void BotConnection::setDebugMode()
{
    sendCommand("debug", logLevel > 0);
}

void BotConnection::keepBotAlive()
{
    sendCommand("ping", logLevel > 2);
}

void BotConnection::sendTarget(Vec2d encoderTarget)
{
    lastSentTarget = encoderTarget;
    stringstream ss;
    ss<<"target "<<encoderTarget.x<<" "<<encoderTarget.y;
    sendCommand(ss.str(), logLevel > 0);
}


void BotConnection::onBotCommPacket(std::string data)
{
    incomingData += data;

    cout << "IncomingData: '" << incomingData << "'" << endl;

    while (cmdLink->readCmd()) {
        char cmd = cmdLink->cmd();

        cout << "BotConnection got cmd: " << cmd << endl;
        continue;

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
            //dataStream >> a >> b;
            updateTarget({a,b});
            break;
        case 'P': // position
            //dataStream >> a >> b >> leftMotor >> rightMotor;
            updateEncoders({a,b});
            updateMotors({leftMotor, rightMotor});
            //cout << recCmd << endl;
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
