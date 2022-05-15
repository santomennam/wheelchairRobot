#include "botcommserial.h"
#include "graphics.h"
#include "simplecrc.h"
#include <iostream>

using namespace std;
using namespace mssm;

BotCommSerial::BotCommSerial(mssm::Graphics &g)
    : g{g}
{

}

void BotCommSerial::handleRawData(std::string data)
{
    client->onBotCommPacket(data);
}

void BotCommSerial::attach(BotCommClient *client)
{
    this->client = client;
}

void BotCommSerial::sendPacket(std::string data)
{
    port.write(data);
}

void BotCommSerial::connect(std::string connectionName)
{
    port.open(connectionName,115200);
}

void BotCommSerial::disconnect()
{
    port.close();
}

