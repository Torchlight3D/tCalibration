#pragma once

#include <QString>

namespace tl {
namespace qnetwork {

QString findDeviceHardwareAddress(const QString &addr, int timeout = 100);

bool ping(const QString &addr, int count = 4, int timeout = 100);

struct Ping
{
    int count{4};
    int timeout{100};

    Ping(int count, int timeout) : count(count), timeout(timeout) {}

    bool operator()(const QString &addr) { return ping(addr, count, timeout); }
};

} // namespace qnetwork
} // namespace tl
