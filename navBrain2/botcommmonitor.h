#ifndef BOTCOMMMONITOR_H
#define BOTCOMMMONITOR_H

#include "botcomm.h"

class BotCommMonitor : public BotComm, public BotCommClient
{
    BotComm *realBot;
    BotCommClient* realClient{nullptr};

public:
    BotCommMonitor(BotComm *realBot);

    // BotCommClient interface
public:
    void onBotCommPacket(std::string data) override;
    void onBotCommConnect() override;
    void onBotCommDisconnect() override;
    void onBotCommError(std::string msg) override;

    // BotComm interface
public:
    void attach(BotCommClient *user) override;
    void sendPacket(std::string data) override;
    bool connect(std::string connectionName) override;
    void disconnect() override;

    // BotComm interface
public:
    bool isConnected() override;
};

#endif // BOTCOMMMONITOR_H
