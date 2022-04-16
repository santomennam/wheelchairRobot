#ifndef BOTCOMM_H
#define BOTCOMM_H

#include <string>

class BotCommClient {
public:
    virtual void onBotCommPacket(std::string data) = 0;
    virtual void onBotCommConnect() = 0;
    virtual void onBotCommDisconnect() = 0;
    virtual void onBotCommError(std::string msg) = 0;
};

class BotComm {
public:
    virtual void attach(BotCommClient* user) = 0;
    virtual void sendPacket(std::string data) = 0;
    virtual void connect(std::string connectionName) = 0;
    virtual void disconnect() = 0;
};


#endif // BOTCOMM_H
