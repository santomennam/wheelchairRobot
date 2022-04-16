#ifndef BOTCOMMSERIAL_H
#define BOTCOMMSERIAL_H

#include "botcomm.h"

namespace mssm {
class Graphics;
}

class BotCommSerial : public BotComm {

    mssm::Graphics& g;
    int             pluginId;
    BotCommUser    *user;
    std::string     incomingData;
public:
    BotCommSerial(mssm::Graphics& g, int pluginId);

    void handleRawData(std::string data);

    // BotComm interface
    void attach(BotCommUser *user) override;
    void sendPacket(std::string data) override;
    void connect(std::string connectionName) override;
    void disconnect() override;
};

#endif // BOTCOMMSERIAL_H
