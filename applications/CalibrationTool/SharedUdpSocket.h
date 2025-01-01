#pragma once

#include <QHostAddress>
#include <QMutex>
#include <QObject>

#include <QCoroCore>

class QUdpSocket;

namespace tl {

class SharedUdpSocket : public QObject
{
    Q_OBJECT

public:
    explicit SharedUdpSocket(QObject* parent = nullptr);

    bool bindTo(const std::string& localAddress, quint16 localPort);
    bool bound() const;
    void close();

    // using ResponseHandler = std::function<void(const QByteArray&)>;
    QCoro::Task<QByteArray> sendRequest(const QByteArray& request,
                                        const std::string& remoteAddress,
                                        quint16 remotePort);

private slots:
    void handleResponse();

private:
    QUdpSocket* m_host;
    QMutex mutex;
    // std::unordered_map<std::string, ResponseHandler> responseHandlers;
    std::unordered_map<std::string, QPromise<QByteArray>*> pendingRequests;
};

} // namespace tl
