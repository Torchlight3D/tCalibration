#pragma once

#include "gui/qtlistwidget.h"
#include "gui/qtledindicator.h"
#include "DeviceInfo.h"

namespace tl {

// | Host address input | SN input | Status |
class DeviceListItemPrivate;
class DeviceListItem : public QtListWidgetItem
{
    Q_OBJECT

public:
    explicit DeviceListItem(int index, QWidget *parent = nullptr);
    ~DeviceListItem();

    bool connected() const;
    void setConnected(bool connected);

    QString hostAddress() const;
    void setHostAddress(const QString &addr);
    void lockHostAddress(bool lock);

    QString serialNumber() const;
    void setSerialNumber(const QString &sn);
    void setSerialNumberPattern(const QString &pattern);
    void lockSerialNumber(bool lock);
    void focusOnSerialNumber();

    DeviceInfo toInfo() const;

    void setLedState(QtLedIndicator::State state);

    void resetState();

signals:
    void hostAddressChanged(const QString &addr);
    void serialNumberChanged(const QString &sn);
    void serialNumberMatched();

private:
    Q_DECLARE_PIMPL(DeviceListItem);
};

} // namespace tl
