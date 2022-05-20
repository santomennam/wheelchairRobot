#include "botconnection.h"


#include <iostream>
#include <iomanip>

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
    cout << "Tank: " << left << " " << right << endl;
    lastCommandSent = "TankSteer";
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

bool BotConnection::inDriveableState() const
{
    switch (botState) {
    case BotState::idle:
    case BotState::tank:
    case BotState::target:
        return true;
    case BotState::noConnect:
    case BotState::sleep:
        return false;
    }
    return false;
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
    lastCommandTime = std::chrono::steady_clock::now();
    isWaitingForResponse = true;
}

void BotConnection::resetBot()
{
    cout << "Sending Wake" << endl;
    lastCommandSent = "Wake";

    cmdLink->sendCmd('W'); // reset
    postSend(false);
}


void BotConnection::keepBotAlive()
{
    cmdLink->sendCmd('P');
    postSend(true);
}

void BotConnection::sendTarget(Vec2d encoderTarget)
{
    cout << "Sending Target: " << encoderTarget.x << " " << encoderTarget.y << endl;
    lastCommandSent = "Target";
    lastSentTarget = encoderTarget;
    cmdLink->sendCmdII('T', encoderTarget.x, encoderTarget.y);
    postSend(false);
}

string BotConnection::stateStr()
{
    switch (botState) {
    case BotState::idle:
        return "Idle";
    case BotState::noConnect:
        return "No Connection";
    case BotState::sleep:
        return "Sleep";
    case BotState::tank:
        return "Tank/Drive";
    case BotState::target:
        return "Target";
    }
    return "UnknownState!";
}


void dumpAsHex(std::string data);


void BotConnection::onBotCommPacket(std::string data)
{
//    if (cmdLink->isDebug()) {
//        cout << "RawIncoming:\n";
//        dumpAsHex(data);
//    }
    incomingData += data;

   //cout << "IncomingData: '" << data << "'" << endl;

    while (cmdLink->readCmd()) {



        char cmd = cmdLink->cmd();

        double leftMotor;
        double rightMotor;

        int32_t leftCount;
        int32_t rightCount;

        switch (cmd) {
        case 'N':
            // no connection
            receivedResponse = "NoConnect";
            cout << "Mode: No Connection" << endl;
            botState = BotState::noConnect;
            break;
        case 'S':
            // modeSleep
            receivedResponse = "SleepMode";
            cout << "Mode: Sleep" << endl;
            botState = BotState::sleep;
            break;
        case 'D':
            // modeDrive
            receivedResponse = "DriveMode";
            cout << "Mode: Drive" << endl;
            botState = BotState::tank;
            break;
        case 'T':
            // modeTarget
            receivedResponse = "TargetMode";
            cout << "Mode: Target" << endl;
            botState = BotState::target;
            break;
        case 'W':
            // modeIdle
            receivedResponse = "WakeMode";
            cout << "Mode: Idle" << endl;
            botState = BotState::idle;
            break;
        case 'I':
            receivedResponse = "Info: " + cmdLink->getStr();
            cout << receivedResponse << endl;
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
        case 'C': // encoder counts
            cmdLink->getParam(leftCount);
            cmdLink->getParam(rightCount);
            //receivedResponse = "Encoders: " + to_string(leftCount) + " " + to_string(rightCount);
            //cout << receivedResponse << endl;
            updateEncoders({(double)leftCount, (double)rightCount});
            break;
        case 'M': // motor values
            cmdLink->getParam(leftMotor);
            cmdLink->getParam(rightMotor);
            updateMotors({leftMotor, rightMotor});
            receivedResponse = "Motors: " + to_string(leftMotor) + " " + to_string(rightMotor);
            cout << receivedResponse << endl;
            break;
        case 'E': // Error
            receivedError = receivedResponse;
            cout << "Error: " << receivedError << endl;
            break;
        default:
            cout << "UNKNOWN RESPONSE: " << cmd << " " << data << endl;
            receivedError = "UNKNOWN RESPONSE: " + string(1, cmd) + " " + data;
            break;
        }
    }

    if (cmdLink->isCorrupt()) {
        cout << "Corrupt communication from forebrain: " << cmdLink->getCorruptMsg() << endl;
        receivedError = "CorruptComm: " + string(cmdLink->getCorruptMsg());
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
