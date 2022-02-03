#include "networkserver.h"
#include "networkclient.h"
#include "networkplugin.h"
#include <iostream>
#include <QThread>

void IdPool::releaseId(int id)
{
    if (id == maxClientId) {
        maxClientId--;
    }
    else {
        releasedClientIds.push_back(id);
    }
}

int IdPool::nextId()
{
    if (releasedClientIds.empty()) {
        return ++maxClientId;
    }
    int id = releasedClientIds.back();
    releasedClientIds.pop_back();
    return id;
}

NetworkServer::NetworkServer(NetworkPlugin& nc, QObject *parent) : QTcpServer(parent), conn(nc)
{
    qRegisterMetaType< QAbstractSocket::SocketState >();
    qRegisterMetaType< QAbstractSocket::SocketError >();
}

NetworkServer::~NetworkServer()
{
    //qDebug() << "NetworkServer destructor\n";
}

void NetworkServer::releaseId(int id)
{
    idPool.releaseId(id);
}

void NetworkServer::socketStateChange(int connectionId, NetworkSocketEvent state, const std::string& msg)
{
    conn.onSocketStateChange(connectionId, state, msg);
}

void NetworkServer::stopServer()
{
    clients.clear();
    close();
}

void NetworkServer::startServer(int port)
{
    if(!this->listen(QHostAddress::Any, port))
    {
        std::cout  << "Could not start server" << std::endl;
    }
    else
    {
        //qDebug() << "Listening to port " << port << "...";
    }
}

// This function is called by QTcpServer when a new connection is available.

void NetworkServer::incomingConnection(qintptr socketDescriptor)
{
    // We have a new connection
    qDebug() << "Socket connecting on thread: " << QThread::currentThreadId();
    int id = idPool.nextId();
    clients.emplace_back(new NetworkClient(id, this, socketDescriptor));

    //socketStateChange(id, NetworkSocketEvent::connected, "Connected");
}

void NetworkServer::closeConnection(int connectionId)
{
    for (auto& client : clients) {
        if (client->id() == connectionId) {
            client->closeSocket();
            return;
        }
    }
}


int NetworkServer::connect(const std::string& host, int port)
{
    // We have a new connection
    qDebug() << "Creating socket connection on thread: " << QThread::currentThreadId();

    int id = idPool.nextId();

    clients.emplace_back(new NetworkClient(id, this, host, port));

    return id;
}


void NetworkServer::receiver(int connectionId, const std::string& data)
{
    conn.receiver(connectionId, data);
}

bool NetworkServer::queueToSend(int connectionId, const std::string& data)
{
    for (auto& client : clients) {
        if (client->id() == connectionId) {
            client->queueToSend(data);
            return true;
        }
    }
    std::cout << "Unable to find client with connectionId: " << connectionId << " to send data: " << data << std::endl;
    std::cout << "Num Clients: " << clients.size() << std::endl;
    return false;
}

void NetworkServer::sendAllQueued()
{
    clients.erase(std::remove_if(clients.begin(), clients.end(), [](std::unique_ptr<NetworkClient>& obj) { return !obj->isConnected(); }), clients.end());

    for (auto& client : clients) {
        client->sendQueued();
    }
}
