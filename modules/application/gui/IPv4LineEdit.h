#pragma once

#include <QWidget>

class QLineEdit;

namespace thoht {

class IPv4AddressValidator;

class IPv4Edit : public QWidget
{
    Q_OBJECT

public:
    explicit IPv4Edit(QWidget *parent = nullptr);
    ~IPv4Edit();

    // FIXME: This api behavior is wrong
    void setIpv4Address(const QString &ip);
    QString ipv4Address() const;

    bool isEditReallyFinished(const QString &ipv4Address) const;

signals:
    void ipAddressChanged();

public slots:
    void updateIpv4Address();

private:
    // TODO: How about directly inherent QLineEdit
    QLineEdit *m_lineedit;
    QString m_ipv4Address;
};

} // namespace thoht
