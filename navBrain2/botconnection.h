#ifndef BOTCONNECTION_H
#define BOTCONNECTION_H

#include "graphics.h"
#include <functional>
#include "botcomm.h"
#include "CmdLink.h"

using tp = std::chrono::time_point<std::chrono::steady_clock>;

enum class BotState {
  tank,
  target,
  idle,
  sleep,
  noConnect
};

class BotConnection : public BotCommClient {

    BotComm* botComm;

    CmdLink* cmdLink;

    BotState botState{BotState::noConnect};

    std::string lastCommandSent;
    tp    lastCommandTime;
    tp    lastResponseTime;
    bool  isWaitingForResponse{false};

    bool  lastCmdPing{false};

    Vec2d lastSentTarget;

    Vec2d lastReceivedMotors;
    Vec2d lastReceivedEncoders;
    Vec2d lastReceivedTarget;

    std::string incomingData;
    std::string receivedResponse;
    std::string receivedError;
    std::string receivedInfo;

    std::function<void(Vec2d targets)>  onTargetUpdate{nullptr};
    std::function<void(Vec2d encoders)> onEncoderUpdate{nullptr};
    std::function<void(Vec2d motors)>   onMotorUpdate{nullptr};
public:
    BotConnection(BotComm* botComm);

    void update();

    void tankSteer(int left, int right); // left/right -1, 0, 1

    std::string getReceivedCommand() const { return receivedResponse; }
    std::string getReceivedError() const { return receivedError; }
    std::string getReceivedInfo() const { return receivedInfo; }

    void setOnTargetUpdateHandler(std::function<void(Vec2d target)>  onTargetUpdate);
    void setOnEncoderUpdateHandler(std::function<void(Vec2d encoder)> onEncoderUpdate);
    void setOnMotorUpdateHandler(std::function<void(Vec2d motors)> onMotorUpdate);

    bool inDriveableState() const;
    bool readyForNextCommand() { return !isWaitingForResponse; }
 //   bool waitingForResponse() { return isWaitingForResponse; }

    void resetBot();
    void setDebug(bool dbg) { cmdLink->setDebug(dbg); }
    void keepBotAlive();
    void sendTarget(Vec2d encoderTarget);

    std::string stateStr();

    std::string lastCommand() const { return lastCommandSent; }

    Vec2d getTarget(bool lastSent) const        { return lastSent ? lastSentTarget : lastReceivedTarget;   }
    Vec2d getMotors() const                     { return lastReceivedMotors;   }
    Vec2d getEncoders() const                   { return lastReceivedEncoders; }

    double elapsedSinceLastSend() const;
    double elapsedSinceLastResponse() const;


private:
    void postSend(bool wasPing);

    void updateEncoders(Vec2d encoderCounts);
    void updateTarget(Vec2d encoderTarget);
    void updateMotors(Vec2d motorSpeeds);

    // BotCommUser interface
public:
    void onBotCommPacket(std::string data) override;
    void onBotCommConnect() override;
    void onBotCommDisconnect() override;
    void onBotCommError(std::string msg) override;
};


#endif // BOTCONNECTION_H
