#include "botcommserial.h"
#include "graphics.h"
#include "simplecrc.h"
#include "serialportreader.h"
#include <iostream>

using namespace std;
using namespace mssm;

BotCommSerial::BotCommSerial(mssm::Graphics &g)
    : g{g}
{

}

void BotCommSerial::handleRawData(std::string data)
{
    std::replace(data.begin(),data.end(),'\r','\n');
    incomingData += data;
    while(true){
        auto i = find(incomingData.begin(),incomingData.end(),'#');
        if(i == incomingData.end())
        {
            return;
        }
        incomingData.erase(incomingData.begin(),i);
        auto j = find(incomingData.begin(),incomingData.end(),'\n');
        if(j == incomingData.end())
        {
            return;
        }

        string recCmd = incomingData.substr(1,(j-incomingData.begin()-1));
        incomingData.erase(incomingData.begin(),j);

        if (!stripCRC8(recCmd)) {
            if (client) {
                client->onBotCommError("CHECKSUM ERROR: " + recCmd);
            }
            continue;
        }

        if (client) {
            client->onBotCommPacket(recCmd);
        }
        else {
            cout << "NO CLIENT TO RECEIVE PACKET!" << endl;
        }
    }
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
        data = wrapDelimitedCRC8(data);
        g.callPlugin(pluginId,static_cast<int>(SerialPortReader::Command::send),0, data);
    }
}

void BotCommSerial::connect(std::string connectionName)
{
    pluginId = g.registerPlugin([connectionName](QObject* parent) {
          return new SerialPortReader(parent, connectionName, QSerialPort::Baud19200);
    });
}

void BotCommSerial::disconnect()
{
}

