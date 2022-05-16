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

    if (pluginId < 0) {
        if (client) {
            client->onBotCommError("Attempt to sendPacket before connected!!");
        }
    }
    else {
  //      g.callPlugin(pluginId,static_cast<int>(SerialPortReader::Command::send),0, data);
    }
}

void BotCommSerial::connect(std::string connectionName)
{
//    pluginId = g.registerPlugin([connectionName](QObject* parent) {
//          return new SerialPortReader(parent, connectionName, QSerialPort::Baud115200);
//    });
}

void BotCommSerial::disconnect()
{
}

