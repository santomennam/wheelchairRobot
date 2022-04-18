#ifndef BOTCONNECTION_H
#define BOTCONNECTION_H

#include "graphics.h"
#include <functional>
#include "botcomm.h"

using tp = std::chrono::time_point<std::chrono::steady_clock>;

// protocol:

// laptop/client sends a command, and does not send another command until the original has been acknowledged
//
// Bot can respond with multiple replies, but the last thing it sends should be an Ack (A) or Nack (N)
// The Ack should contain the CRC of the command it is acknowledging
//
// Example:
//  Laptop sends:    #BF:tank 1 1
//  Bot sends:       #13:P 324 2223           <- encoder counts
//                   #4E:I Some Information   <- debug information
//                   #A3:M 25 25              <- motor values
//                   #78:A BF                 <- ack for tank command
//
//  Laptop sends:    #E1:ping
//  Bot sends:       #13:P 324 2223           <- encoder counts
//                   #78:A E1                 <- ack for ping command
//
//  Laptop sends:    #E1:ping
//  Bot sends:       #13:P 324 2223           <- encoder counts
//                   #78:A E1                 <- ack for ping command
//
//  Laptop sends:    #E1:garbage
//  Bot sends:       #13:E Unknown Command garbage
//                   #78:N E1                 <- nack (unrecognized command)


class BotConnection : public BotCommClient {

    BotComm* botComm;

    std::string lastCommandSent;
    tp    lastCommandTime;
    tp    lastResponseTime;
    bool  isWaitingForResponse{false};

    int   logLevel{0};
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

    void tankSteer(double left, double right); // left/right in range -1 to 1

    std::string getReceivedCommand() const { return receivedResponse; }
    std::string getReceivedError() const { return receivedError; }
    std::string getReceivedInfo() const { return receivedInfo; }

    void setOnTargetUpdateHandler(std::function<void(Vec2d target)>  onTargetUpdate);
    void setOnEncoderUpdateHandler(std::function<void(Vec2d encoder)> onEncoderUpdate);
    void setOnMotorUpdateHandler(std::function<void(Vec2d motors)> onMotorUpdate);

    bool readyForNextCommand() { return !isWaitingForResponse; }
    bool waitingForResponse() { return isWaitingForResponse; }

    void toggleLogging() { logLevel = (logLevel + 1) % 4; }

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
    double elapsedSinceLastResponse() const;


private:
    void sendCommand(std::string cmd, bool debug);

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
