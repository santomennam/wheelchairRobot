#ifndef BOTCOMMSERIAL_H
#define BOTCOMMSERIAL_H
#include "serialport.h"
#include "botcomm.h"

namespace mssm {
class Graphics;
}

class BotCommSerial : public BotComm {

    mssm::Graphics& g;
//    int             pluginId{-1};
    SerialPort port;
    BotCommClient  *client;
    std::string     incomingData;
public:
    BotCommSerial(mssm::Graphics& g);

    void update();
//    int getPluginId() const { return pluginId; }

    // BotComm interface
    void attach(BotCommClient *client) override;
    void sendPacket(std::string data) override;
    void connect(std::string connectionName) override;
    void disconnect() override;
};

#endif // BOTCOMMSERIAL_H
