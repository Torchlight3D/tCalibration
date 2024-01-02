#include "EthernetAdaptorHelper.h"

#include <QGridLayout>
#include <QLabel>
#include <QNetworkInterface>
#include <QPushButton>

#include "gui/qstringutils.h"

namespace thoht {

EthernetAdaptorHelper::EthernetAdaptorHelper(QWidget* parent) : QDialog(parent)
{
    setWindowTitle(tr("Ethernet Adaptor Helper"));

    auto m_connectDeviceTips = new QLabel(this);
    m_connectDeviceTips->setText(tr("Plug new ethernet adaptor in PC."));

    auto m_removeDeviceTips = new QLabel(this);
    m_removeDeviceTips->setText(tr("Remove the new ethernet adaptor."));

    auto m_resultLabel = new QLabel(this);
    m_resultLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

    auto m_confirmConnectBtn = new QPushButton(tr("Confirm Pluged in"));
    auto m_confirmRemoveBtn = new QPushButton(tr("Confirm Removed"));

    auto layout = new QGridLayout(this);
    layout->addWidget(m_connectDeviceTips, 0, 0);
    layout->addWidget(m_confirmConnectBtn, 0, 1);
    layout->addWidget(m_removeDeviceTips, 1, 0);
    layout->addWidget(m_confirmRemoveBtn, 1, 1);
    layout->addWidget(m_resultLabel, 2, 0);

    connect(m_confirmConnectBtn, &QAbstractButton::clicked, this, [=, this]() {
        const auto interfaces = QNetworkInterface::allInterfaces();

        m_lastInterfaces.clear();
        for (const auto& interface : interfaces) {
            if (interface.type() == QNetworkInterface::Ethernet) {
                m_lastInterfaces.push_back(interface.hardwareAddress());
            }
        }

        m_removeDeviceTips->show();
        m_confirmRemoveBtn->show();
    });
    connect(m_confirmRemoveBtn, &QAbstractButton::clicked, this, [=, this]() {
        const auto interfaces = QNetworkInterface::allInterfaces();

        for (const auto& interface : interfaces) {
            const auto hardwareAddr = interface.hardwareAddress();
            if (interface.type() == QNetworkInterface::Ethernet &&
                m_lastInterfaces.contains(hardwareAddr)) {
                m_lastInterfaces.removeAll(hardwareAddr);
            }
        }

        if (m_lastInterfaces.size() != 1) {
            m_resultLabel->setText(
                tr("Failed to find new ethernet adaptor hardware address."));
        }
        else {
            auto hardwareAddressToIP = [](const QString& mac) -> QString {
                // Dont need to check, because the input is from Qt
                const auto parts = mac.split(':');
                const auto lastThree = parts.last(3);
                bool ok;
                return u"9.%1.%2.%3"_s.arg(
                    QString::number(lastThree[0].toInt(&ok, 16)),
                    QString::number(lastThree[1].toInt(&ok, 16)),
                    QString::number(lastThree[2].toInt(&ok, 16)));
            };

            m_resultLabel->setText(
                tr("New ethernet adaptor infos:."
                   "\n"
                   "Hardware address: %1"
                   "\n"
                   "Corresponding IP address: %2")
                    .arg(m_lastInterfaces.front(),
                         hardwareAddressToIP(m_lastInterfaces.front())));
        }
    });
}

} // namespace thoht

#include "moc_EthernetAdaptorHelper.cpp"
