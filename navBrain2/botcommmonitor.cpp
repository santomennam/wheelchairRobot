#include "botcommmonitor.h"

BotCommMonitor::BotCommMonitor(BotComm *realBot)
    : realBot{realBot}
{

}

// BotCommClient interface

void BotCommMonitor::onBotCommPacket(std::string data)
{
    realClient->onBotCommPacket(data);
}

void BotCommMonitor::onBotCommConnect()
{
    realClient->onBotCommConnect();
}

void BotCommMonitor::onBotCommDisconnect()
{
    realClient->onBotCommDisconnect();
}

void BotCommMonitor::onBotCommError(std::string msg)
{
    realClient->onBotCommError(msg);
}


// BotComm interface

void BotCommMonitor::attach(BotCommClient *user)
{
    realClient = user;
    realBot->attach(this);
}

void BotCommMonitor::sendPacket(std::string data)
{
    realBot->sendPacket(data);
}

void BotCommMonitor::connect(std::string connectionName)
{
    realBot->connect(connectionName);
}

void BotCommMonitor::disconnect()
{
    realBot->disconnect();
}
