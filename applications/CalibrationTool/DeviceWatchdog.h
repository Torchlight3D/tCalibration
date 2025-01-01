#pragma once

#include <QObject>

#include "gui/qtthread.h"

namespace tl {

class DeviceWatchdog : public QObject
{
    Q_OBJECT

public:
    using QObject::QObject;
    ~DeviceWatchdog();

    void setAddresses(const QStringList &addresses);

signals:
    void statusUpdated(const QList<bool> &status);

public slots:
    void startWatch();
    void stopWatch();

private:
    QStringList m_hostAddresses;
    std::atomic<bool> m_running;
    std::unique_ptr<QTimer> m_timer;
};

class DeviceWatcher : public QtThread<DeviceWatchdog>
{
    Q_OBJECT

    using Base = QtThread<DeviceWatchdog>;

public:
    explicit DeviceWatcher(QObject *parent = nullptr);

signals:
    void setAddresses(const QStringList &addresses);
    void startWatch();
    void stopWatch();

    void statusUpdated(const QList<bool> &status);
};

} // namespace tl
