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

void BotConnection::tankSteer(int left, int right)
{
    cmdLink->sendCmdBB('D', left, right); // drive
    postSend(false);
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

void BotConnection::postSend(bool wasPing)
{
    lastCmdPing = wasPing;

    if (isWaitingForResponse) {
        cout << ">>>>>>>>>>>>>>> sendCommand didn't wait for response <<<<<<<<<<<<<" << endl;
    }
    //lastCommandSent = cmd;
    lastCommandTime = std::chrono::steady_clock::now();
    isWaitingForResponse = true;
}

void BotConnection::resetBot()
{
    cmdLink->sendCmd('R'); // reset
    postSend(false);
}


void BotConnection::keepBotAlive()
{
    cmdLink->sendCmd('P');
    postSend(true);
}

void BotConnection::sendTarget(Vec2d encoderTarget)
{
    lastSentTarget = encoderTarget;
    cmdLink->sendCmdII('T', encoderTarget.x, encoderTarget.y);
    postSend(false);
}


void BotConnection::onBotCommPacket(std::string data)
{
    incomingData += data;

   // cout << "IncomingData: '" << data << "'" << endl;

    while (cmdLink->readCmd()) {
        char cmd = cmdLink->cmd();

        double a;
        double b;

        double leftMotor;
        double rightMotor;

        char buffer[10];

        int32_t leftCount;
        int32_t rightCount;

        char leftM;
        char rightM;

        switch (cmd) {
        case 'I':
          //  cout << "Got Info: " << endl;
            receivedResponse = "Info: " + cmdLink->getStr();
            if (receivedResponse != "Info: Ping") {
                cout << receivedResponse << endl;
            }
            break;
        case 'K':
            // Ack: Acknowledge
            lastResponseTime = std::chrono::steady_clock::now();
            isWaitingForResponse = false;
            //cout << "Got Ack" << endl;
            break;
        case 'k':
            // Nack: Negative Acknowledge
            lastResponseTime = std::chrono::steady_clock::now();
            isWaitingForResponse = false;
            cout << "NACKNACKNACK" << endl;
            break;
//        case 'T': // target ack
//            //dataStream >> a >> b;
//            updateTarget({a,b});
//            break;
        case 'C': // encoder counts
            cmdLink->getParam(leftCount);
            cmdLink->getParam(rightCount);
            cout << "Counts: " << leftCount << " " << rightCount << endl;
            updateEncoders({(double)leftCount, (double)rightCount});
        case 'M': // motor values
            cmdLink->getParam(leftMotor);
            cmdLink->getParam(rightMotor);
            updateMotors({leftMotor, rightMotor});
            break;
        case 'E': // Error
            receivedError = receivedResponse;
            break;
        case 'X': // EStop
            break;
        default:

            cout << "UNKNOWN RESPONSE: " << cmd << " " << data << endl;
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
