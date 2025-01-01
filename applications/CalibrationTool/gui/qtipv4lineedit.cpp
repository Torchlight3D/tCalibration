#include "qtipv4lineedit.h"

#include <QBoxLayout>
#include <QHostAddress>
#include <QLineEdit>
#include <QValidator>

namespace tl {

using namespace Qt::Literals::StringLiterals;

namespace {

inline bool isIPAddressValid(const QString &addr)
{
    return QHostAddress{}.setAddress(addr);
}

} // namespace

class IPv4AddressValidator final : public QValidator
{
    Q_OBJECT

public:
    using QValidator::QValidator;

    State validate(QString &input, int &pos) const override;
};

// Explanation:
// This validator checks
// 1. Whether each subnet is not more than 255.
// 2. '0' in the subnet means the next digit cannot be filled in the same
// subnet.
// 3. Space between two digits is allowed, e.g.1<space>3 is considred as 13.
// 4. Numbers start with zero is not allowed, e.g. 00 or 01 or 02.. is not
// allowed.
//
// FIXME:
// 1. "9.01.123.10" should not be accepted
QValidator::State IPv4AddressValidator::validate(QString &input, int &pos) const
{
    // backspace -- very beginning
    if (pos == 0) {
        return Acceptable;
    }

    // crossing '.' delimeter
    const auto prevPos = pos - (pos == 4 || pos == 8 || pos == 12 ? 2 : 1);

    // Is it a space
    if (input.at(prevPos).isDigit()) {
        int digit = input.at(prevPos).digitValue();
        if (prevPos != 0 && prevPos != 4 && prevPos != 8) {
            // the 2nd pos in each subnet
            if (prevPos == 1 || prevPos == 9 || prevPos == 13) {
                // MSB in its subnet
                if (input.at(prevPos - 1).isDigit()) {
                    if (input.at(prevPos - 1).digitValue() == 0) {
                        // can't have 0 and number
                        return Invalid;
                    }
                }
            }
            // this means it is the last pos in each subnet
            else {
                int tensplaceVal = 0;
                int hundredsplaceVal = 0;
                if (input.at(prevPos - 2).isDigit()) {
                    if (input.at(prevPos - 2).digitValue() == 0) {
                        // can't have 0 and number
                        return Invalid;
                    }

                    // check the middle is present or not
                    if (input.at(prevPos - 1).isDigit()) {
                        tensplaceVal = input.at(prevPos - 1).digitValue() * 10;
                    }
                    else {
                        if (input.at(prevPos - 2).isDigit()) {
                            // 1 space 3 is actually 13.
                            tensplaceVal =
                                input.at(prevPos - 2).digitValue() * 10;
                            // no hundreds place
                            hundredsplaceVal = -1;
                        }
                    }
                    if (input.at(prevPos - 2).isDigit() &&
                        hundredsplaceVal != -1) {
                        hundredsplaceVal =
                            input.at(prevPos - 2).digitValue() * 100;
                    }
                    int number = hundredsplaceVal + tensplaceVal + digit;
                    if (number < 0 || number > 255) {
                        return Invalid;
                    }
                }
            }
        }
    }

    return Acceptable;
}

class QtIPv4LineEditPrivate
{
    Q_DEFINE_PIMPL(QtIPv4LineEdit)

public:
    explicit QtIPv4LineEditPrivate(QtIPv4LineEdit *q);

    void init();

public:
    // TODO: How about directly inherent QLineEdit
    QLineEdit *m_input;
};

QtIPv4LineEditPrivate::QtIPv4LineEditPrivate(QtIPv4LineEdit *q)
    : q_ptr(q), m_input(new QLineEdit(q))
{
}

void QtIPv4LineEditPrivate::init()
{
    Q_Q(QtIPv4LineEdit);
    // q->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    // q->setFixedWidth(100);

    m_input->setInputMask(u"000.000.000.000"_s);
    m_input->setValidator(new IPv4AddressValidator(q));
    m_input->setCursorPosition(0);

    auto layout = new QVBoxLayout(q);
    layout->setContentsMargins({});
    layout->addWidget(m_input);

    q->setConnected(false);
}

QtIPv4LineEdit::QtIPv4LineEdit(QWidget *parent)
    : QWidget(parent), d_ptr(new QtIPv4LineEditPrivate(this))
{
    Q_D(QtIPv4LineEdit);
    d->init();

    connect(d->m_input, &QLineEdit::editingFinished, this, [this, d]() {
        if (const auto str = d->m_input->text(); isIPAddressValid(str))
            emit addressChanged(str);
    });
}

QtIPv4LineEdit::~QtIPv4LineEdit() = default;

void QtIPv4LineEdit::setAddress(const QString &str)
{
    if (str.isEmpty()) {
        return;
    }

    Q_D(QtIPv4LineEdit);

    if (!isIPAddressValid(str)) {
        return;
    }

    {
        QSignalBlocker blocker{d->m_input};
        d->m_input->setText(str);
    }
}

QString QtIPv4LineEdit::address() const
{
    Q_D(const QtIPv4LineEdit);
    if (const auto addr = d->m_input->text().remove(' ');
        isIPAddressValid(addr)) {
        return addr;
    }

    return {};
}

void QtIPv4LineEdit::setConnected(bool on)
{
    Q_D(QtIPv4LineEdit);
    d->m_input->setStyleSheet(on ? u"QLineEdit { background-color: #66C966; }"_s
                                 : QString{});
}

void QtIPv4LineEdit::clear()
{
    Q_D(const QtIPv4LineEdit);
    QSignalBlocker blocker{d->m_input};
    d->m_input->clear();
}

} // namespace tl

#include "qtipv4lineedit.moc"
#include "moc_qtipv4lineedit.cpp"
