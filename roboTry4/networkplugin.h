#ifndef NETWORKCONNECTION_H
#define NETWORKCONNECTION_H


#include "plugin.h"
#include <vector>
#include <memory>
#include "networkserver.h"

namespace mssm {
class Graphics;
class Event;
}

class NetworkPlugin : public mssm::Plugin
{
    Q_OBJECT

    class NetworkEvent {
    public:
        int id;
        NetworkSocketEvent state;
        std::string data;
    };

    int serverPort{0};
    std::unique_ptr<NetworkServer> server;

    bool started{false};
    bool closed{false};

    std::vector<NetworkEvent> networkEvents;

public:
    static constexpr int CMD_CONNECT = 1;  // arg2 = port arg3 = hostname
    static constexpr int CMD_SEND    = 2;  // arg2 = client id  arg3 = data
    static constexpr int CMD_CLOSE_PLUGIN = 3; // shut down the plugin itself
    static constexpr int CMD_CLOSE   = 4;  // arg2 = client id

    explicit NetworkPlugin(QObject *parent, int serverPort = 0);
    virtual ~NetworkPlugin();

    void startServer();  // do not call from graphicsMain thread
    bool shouldDelete() override;
    void update(std::function<void(int, int, int, const std::string&)> sendEvent) override;
    void call(int arg1, int arg2, const std::string& arg3) override;

    void closePlugin() { closed = true; }

    void receiver(int connectionId, const std::string& data);

    void onSocketStateChange(int connectionId, NetworkSocketEvent state, const std::string& msg);
};

class NetworkServerPlugin {
public:

protected:
    mssm::Graphics& g;

    int networkPluginId;
    int port;

public:
    NetworkServerPlugin(mssm::Graphics& g, int port);

    bool handleEvent(const mssm::Event& evt, NetworkSocketEvent& netEventType, int& clientId, std::string& data);
    void send(int clientId, const std::string& data);
    void close(int clientId);
    int  pluginId() { return networkPluginId; }

    void closePlugin();
};

class NetworkClientPlugin {

    enum class NetworkClientState {
        beforeConnect,
        connecting,
        connected,
        disconnected
    };

private:
    mssm::Graphics& g;

    NetworkClientState state;

    int networkPluginId;
    int socketId;

    int port;
    std::string hostname;
public:
    NetworkClientPlugin(mssm::Graphics& g, int port, const std::string& hostname);

    void reConnect(int port, const std::string& hostname);
    bool handleEvent(const mssm::Event& evt, NetworkSocketEvent& socketEvent, std::string& data);
    bool send(const std::string& data);
    void closeConnection();
    bool hasConnected() const { return state == NetworkClientState::connected; } // note!  false does not mean that we have been disconnected (because we may not have yet connected)
    bool wasDisconnected() const { return state == NetworkClientState::disconnected; }
    int  pluginId() const { return networkPluginId; }

    void closePlugin();
};

#endif // NETWORKCONNECTION_H
