#pragma once

#include <QDialog>

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
QT_END_NAMESPACE

namespace thoht {

class EthernetAdaptorHelper : public QDialog
{
    Q_OBJECT

public:
    explicit EthernetAdaptorHelper(QWidget* parent = nullptr);

private:
    QStringList m_lastInterfaces;
};

} // namespace thoht
