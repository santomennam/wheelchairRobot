#ifndef NETWORKSEVER_H
#define NETWORKSEVER_H

#include <QTcpServer>
#include <memory>

class NetworkPlugin;
class NetworkClient;

enum class NetworkSocketEvent {
    disconnected,
    connected,
    data,
    error,
    other
};

class IdPool {
private:
    std::vector<int> releasedClientIds;   // client ids are reused to keep the indices low
    int maxClientId {0};
public:
    int nextId(); // first ID will be 1
    void releaseId(int id);
};

class NetworkServer : public QTcpServer
{
    Q_OBJECT

    IdPool idPool;

    std::vector<std::unique_ptr<NetworkClient>> clients;

    NetworkPlugin& conn;

public:
    explicit NetworkServer(NetworkPlugin& conn, QObject *parent = 0);
    ~NetworkServer();

    void releaseId(int id);
    int  getAvailableId();
    void startServer(int port);
    void stopServer();

signals:

public slots:

protected:

    void incomingConnection(qintptr socketDescriptor);
public:
    int  connect(const std::string& host, int port);
    void receiver(int connectionId, const std::string& data);
    bool queueToSend(int connectionId, const std::string& data);
    void sendAllQueued();
    void socketStateChange(int connectionId, NetworkSocketEvent state, const std::string& msg);
    void closeConnection(int connectionId);
};

#endif // NETWORKSEVER_H
