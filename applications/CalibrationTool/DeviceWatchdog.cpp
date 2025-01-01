#include "DeviceWatchdog.h"

#include <QTimer>
#include <QtConcurrent/QtConcurrent>

#include "gui/qnetworkutils.h"

namespace tl {

DeviceWatchdog::~DeviceWatchdog() { stopWatch(); }

void DeviceWatchdog::setAddresses(const QStringList &addresses)
{
    if (addresses.empty()) {
        return;
    }

    m_hostAddresses = addresses;
}

void DeviceWatchdog::startWatch()
{
    if (m_running) {
        return;
    }

    m_running = true;

    if (!m_timer) {
        m_timer = std::make_unique<QTimer>();
        connect(m_timer.get(), &QTimer::timeout, this, [this]() {
            if (!m_running) {
                return;
            }

            if (m_hostAddresses.empty()) {
                return;
            }

            const auto status = QtConcurrent::blockingMapped(
                m_hostAddresses, qnetwork::Ping{1, 200});

            emit statusUpdated(status);
        });
    }

    m_timer->start(1000);
}

void DeviceWatchdog::stopWatch()
{
    if (!m_running) {
        return;
    }

    m_running = false;
    if (m_timer) {
        m_timer->stop();
    }
}

DeviceWatcher::DeviceWatcher(QObject *parent)
    : Base(new DeviceWatchdog(), parent)
{
    connect(this, &DeviceWatcher::setAddresses, worker(),
            &DeviceWatchdog::setAddresses);
    connect(this, &DeviceWatcher::startWatch, worker(),
            &DeviceWatchdog::startWatch);
    connect(this, &DeviceWatcher::stopWatch, worker(),
            &DeviceWatchdog::stopWatch);

    connect(worker(), &DeviceWatchdog::statusUpdated, this,
            &DeviceWatcher::statusUpdated);
}

} // namespace tl

#include "moc_DeviceWatchdog.cpp"
