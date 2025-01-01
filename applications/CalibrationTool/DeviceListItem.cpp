#include "DeviceListItem.h"

#include <QBoxLayout>
#include <QCoreApplication>
#include <QLabel>
#include <QLineEdit>
#include <QRegularExpression>

#include "gui/qtipv4lineedit.h"
#include "gui/qtledindicator.h"

namespace tl {

using namespace Qt::Literals::StringLiterals;

class DeviceListItemPrivate
{
    Q_DECLARE_TR_FUNCTIONS(tl::DeviceListItem)
    Q_DEFINE_PIMPL(DeviceListItem);

public:
    explicit DeviceListItemPrivate(DeviceListItem *q);

    void init(int index);

public:
    QtIPv4LineEdit *m_hostAddr{nullptr};
    QLineEdit *m_sn{nullptr};
    QRegularExpression m_snRegex;
    QtLedIndicator *m_state{nullptr};
    bool m_connected{false};
};

DeviceListItemPrivate::DeviceListItemPrivate(DeviceListItem *q)
    : q_ptr(q),
      m_hostAddr(new QtIPv4LineEdit(q)),
      m_sn(new QLineEdit(q)),
      m_state(new QtLedIndicator(q))
{
}

void DeviceListItemPrivate::init(int index)
{
    Q_Q(DeviceListItem);
    q->setToolTip(tr("Device status"));
    q->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    m_sn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    m_sn->setAlignment(Qt::AlignCenter);
    m_sn->setClearButtonEnabled(true);
    m_sn->setPlaceholderText(
        tr("Device %1 SN").arg(QString::number(index + 1)));
    m_sn->setInputMethodHints(Qt::ImhLatinOnly);
    // m_sn->setFixedWidth(90);

    m_state->setSize({35, 20});
    m_state->setFlat(true);

    auto layout = new QHBoxLayout(q);
    layout->setContentsMargins({});
    layout->addWidget(m_hostAddr);
    layout->addWidget(m_sn);
    layout->addWidget(m_state);
}

DeviceListItem::DeviceListItem(int index, QWidget *parent)
    : QtListWidgetItem(parent), d_ptr(new DeviceListItemPrivate(this))
{
    Q_D(DeviceListItem);
    d->init(index);

    connect(d->m_hostAddr, &QtIPv4LineEdit::addressChanged, this,
            &DeviceListItem::hostAddressChanged);
    connect(d->m_sn, &QLineEdit::textChanged, this,
            &DeviceListItem::serialNumberChanged);
    connect(d->m_sn, &QLineEdit::textEdited, this,
            [this, d](const QString &text) {
                if (d->m_snRegex.match(text).hasMatch()) {
                    emit serialNumberMatched();
                }
            });
}

DeviceListItem::~DeviceListItem() = default;

bool DeviceListItem::connected() const
{
    Q_D(const DeviceListItem);
    return d->m_connected;
}

void DeviceListItem::setConnected(bool connected)
{
    Q_D(DeviceListItem);
    d->m_hostAddr->setConnected(connected);
    d->m_connected = connected;
}

QString DeviceListItem::hostAddress() const
{
    Q_D(const DeviceListItem);
    return d->m_hostAddr->address();
}

void DeviceListItem::setHostAddress(const QString &addr)
{
    Q_D(DeviceListItem);
    d->m_hostAddr->setAddress(addr);
}

void DeviceListItem::lockHostAddress(bool lock)
{
    Q_D(DeviceListItem);
    d->m_hostAddr->setEnabled(!lock);
}

QString DeviceListItem::serialNumber() const
{
    Q_D(const DeviceListItem);
    return d->m_sn->text();
}

void DeviceListItem::setSerialNumber(const QString &sn)
{
    Q_D(DeviceListItem);
    d->m_sn->setText(sn);
}

void DeviceListItem::setSerialNumberPattern(const QString &pattern)
{
    Q_D(DeviceListItem);
    d->m_snRegex.setPattern(pattern);
}

void DeviceListItem::lockSerialNumber(bool lock)
{
    Q_D(DeviceListItem);
    d->m_sn->setEnabled(!lock);
}

void DeviceListItem::focusOnSerialNumber()
{
    Q_D(DeviceListItem);
    d->m_sn->setFocus();
    d->m_sn->clear();
}

DeviceInfo DeviceListItem::toInfo() const
{
    Q_D(const DeviceListItem);
    return {.name = {},
            .hostAddr = d->m_hostAddr->address().toStdString(),
            .hardwareAddr = {},
            .sn = d->m_sn->text().toStdString(),
            .isVirtual = !d->m_connected};
}

void DeviceListItem::setLedState(QtLedIndicator::State state)
{
    Q_D(DeviceListItem);
    d->m_state->setState(state);
}

void DeviceListItem::resetState()
{
    Q_D(DeviceListItem);
    setConnected(false);
    d->m_hostAddr->setEnabled(true);
    d->m_sn->setEnabled(true);
    d->m_state->setStatus(QtLedIndicator::Status::Off);
}

} // namespace tl

#include "moc_DeviceListItem.cpp"
