#include "SharedUdpSocket.h"

#include <QUdpSocket>
#include <QCoroAbstractSocket>

namespace tl {

SharedUdpSocket::SharedUdpSocket(QObject* parent)
    : QObject(parent), m_host(new QUdpSocket(this))
{
    connect(m_host, &QUdpSocket::readyRead, this,
            &SharedUdpSocket::handleResponse);
}

bool SharedUdpSocket::bindTo(const std::string& localAddr, quint16 localPort)
{
    if (bound()) {
        m_host->close();
    }

    if (const QHostAddress hostAddr{QString::fromStdString(localAddr)};
        !m_host->bind(hostAddr, localPort)) {
        qWarning() << "Failed to bind shared socket to " << hostAddr << " at "
                   << localPort;
        return false;
    }

    return true;
}

bool SharedUdpSocket::bound() const
{
    return m_host->state() == QUdpSocket::BoundState;
}

void SharedUdpSocket::close() { m_host->close(); }

QCoro::Task<QByteArray> SharedUdpSocket::sendRequest(
    const QByteArray& diagram, const std::string& hostAddr, quint16 hostPort)
{
    if (!bound()) {
        co_return {};
    }

    if (const auto bytesSent = m_host->writeDatagram(
            diagram, QHostAddress{QString::fromStdString(hostAddr)}, hostPort);
        bytesSent == -1) {
        qWarning() << "Send request error: " << m_host->errorString();
        co_return {};
    }

    // const auto success = co_await qCoro(m_host).waitForReadyRead(1000);

    // QPromise<bool> promise;
    // pendingRequests.insert({hostAddr, &promise});

    // co_return co_await promise.future();

    // // Send the request
    // m_host->writeDatagram(request, host, port);

    QPromise<QByteArray> promise;
    promise.future();
    {
        QMutexLocker locker(&mutex);
        pendingRequests.insert({hostAddr, &promise});
    }

    co_return co_await promise.future();
}

void SharedUdpSocket::handleResponse()
{
    while (m_host->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(m_host->pendingDatagramSize());

        QHostAddress sender;
        quint16 senderPort;
        [[maybe_unused]] const auto bytesRead = m_host->readDatagram(
            datagram.data(), datagram.size(), &sender, &senderPort);

        const auto ip = sender.toString().toStdString();
        {
            QMutexLocker locker(&mutex);
            if (!pendingRequests.contains(ip)) {
                continue;
            }

            pendingRequests.at(ip)->addResult(datagram);
            pendingRequests.erase(ip);
        }
    }
}

} // namespace tl
