#pragma once

#include <QWidget>

#include "qtcoreutils.h"

namespace tl {

class QtIPv4LineEditPrivate;
class QtIPv4LineEdit : public QWidget
{
    Q_OBJECT

public:
    explicit QtIPv4LineEdit(QWidget *parent = nullptr);
    ~QtIPv4LineEdit();

    void setAddress(const QString &address);
    QString address() const;

    void setConnected(bool on);

public slots:
    void clear();

signals:
    void addressChanged(const QString &address);

private:
    Q_DECLARE_PIMPL(QtIPv4LineEdit)
};

} // namespace tl
