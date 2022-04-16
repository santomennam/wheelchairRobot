#ifndef BOTCONNECTION_H
#define BOTCONNECTION_H

#include "graphics.h"
#include <functional>
#include "botcomm.h"

using tp = std::chrono::time_point<std::chrono::steady_clock>;

class BotConnection : public BotCommClient {

    BotComm* botComm;

    std::string lastCommandSent;
    tp    lastCommandTime;

    Vec2d lastSentTarget;

    Vec2d lastReceivedMotors;
    Vec2d lastReceivedEncoders;
    Vec2d lastReceivedTarget;

    std::string incomingData;
    std::string receivedCommand;
    std::string receivedError;
    std::string receivedInfo;

    std::function<void(Vec2d targets)>  onTargetUpdate{nullptr};
    std::function<void(Vec2d encoders)> onEncoderUpdate{nullptr};
    std::function<void(Vec2d motors)>   onMotorUpdate{nullptr};
public:
    BotConnection(BotComm* botComm);

    void update();

    void tankSteer(double left, double right); // left/right in range -1 to 1

    std::string getReceivedCommand() const { return receivedCommand; }
    std::string getReceivedError() const { return receivedError; }
    std::string getReceivedInfo() const { return receivedInfo; }

    void setOnTargetUpdateHandler(std::function<void(Vec2d target)>  onTargetUpdate);
    void setOnEncoderUpdateHandler(std::function<void(Vec2d encoder)> onEncoderUpdate);
    void setOnMotorUpdateHandler(std::function<void(Vec2d motors)> onMotorUpdate);

    void sendCommand(std::string cmd, bool debug);
    void resetBot();
    void ask();
    void setDebugMode();
    void keepBotAlive();
    void sendTarget(Vec2d encoderTarget);

    std::string lastCommand() const { return lastCommandSent; }

    Vec2d getTarget(bool lastSent) const        { return lastSent ? lastSentTarget : lastReceivedTarget;   }
    Vec2d getMotors() const                     { return lastReceivedMotors;   }
    Vec2d getEncoders() const                   { return lastReceivedEncoders; }

    double elapsedSinceLastSend() const;

private:
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
