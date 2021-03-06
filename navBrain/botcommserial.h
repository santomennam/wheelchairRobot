#ifndef BOTCOMMSERIAL_H
#define BOTCOMMSERIAL_H

#include "botcomm.h"

namespace mssm {
class Graphics;
}

class BotCommSerial : public BotComm {

    mssm::Graphics& g;
    int             pluginId{-1};
    BotCommClient  *client;
    std::string     incomingData;
public:
    BotCommSerial(mssm::Graphics& g);

    void handleRawData(std::string data);
    int getPluginId() const { return pluginId; }

    // BotComm interface
    void attach(BotCommClient *client) override;
    void sendPacket(std::string data) override;
    void connect(std::string connectionName) override;
    void disconnect() override;
};

#endif // BOTCOMMSERIAL_H
