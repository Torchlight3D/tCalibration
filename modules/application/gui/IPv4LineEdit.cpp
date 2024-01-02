#include "IPv4LineEdit.h"

#include <QLineEdit>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QValidator>

namespace thoht {

class IPv4AddressValidator : public QValidator
{
    Q_OBJECT
public:
    using QValidator::QValidator;

    State validate(QString &input, int &pos) const override;
};

// Explanation:
// This validator checks whether each subnet is not more than 255. Also, a zero
// in the subnet means the next digit cannot be filled in the same subnet.
// Space between two digits is allowed, e.g.1<space>3 is considred as 13.
// Meanwhile, numbers start with zero is not allowed, e.g. 00 or 01 or 02.. is
// not allowed.
QValidator::State IPv4AddressValidator::validate(QString &input, int &pos) const
{
    int prevCursorPos{0};
    // Backspace -- very beginning
    if (pos == 0) {
        return Acceptable;
    }

    const bool crossDotDelimeter = pos == 4 || pos == 8 || pos == 12;
    if (crossDotDelimeter) {
        prevCursorPos = pos - 2;
    }
    else {
        prevCursorPos = pos - 1;
    }

    // If it's a space
    if (input.at(prevCursorPos).isDigit()) {
        int digit = input.at(prevCursorPos).digitValue();
        if (prevCursorPos != 0 && prevCursorPos != 4 && prevCursorPos != 8) {
            // The 2nd pos in each subnet
            if (prevCursorPos == 1 || prevCursorPos == 9 ||
                prevCursorPos == 13) {
                // MSB in its subnet
                if (input.at(prevCursorPos - 1).isDigit()) {
                    if (input.at(prevCursorPos - 1).digitValue() == 0) {
                        // Not allowed 0 and number
                        return Invalid;
                    }
                }
            }
            // This branch means it is the last pos in each subnet
            else {
                int tensplaceVal{0};
                int hundredsplaceVal{0};
                if (input.at(prevCursorPos - 2).isDigit()) {
                    if (input.at(prevCursorPos - 2).digitValue() == 0) {
                        // Not allowed 0 and number
                        return Invalid;
                    }

                    // Check the middle is present or not
                    if (input.at(prevCursorPos - 1).isDigit()) {
                        tensplaceVal =
                            input.at(prevCursorPos - 1).digitValue() * 10;
                    }
                    else {
                        if (input.at(prevCursorPos - 2).isDigit()) {
                            // e.g. "1 3" is actually 13.
                            tensplaceVal =
                                input.at(prevCursorPos - 2).digitValue() * 10;
                            // No hundreds place
                            hundredsplaceVal = -1;
                        }
                    }
                    if (input.at(prevCursorPos - 2).isDigit() &&
                        hundredsplaceVal != -1) {
                        hundredsplaceVal =
                            input.at(prevCursorPos - 2).digitValue() * 100;
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

IPv4Edit::IPv4Edit(QWidget *parent)
    : QWidget(parent), m_lineedit(new QLineEdit(this))
{
    m_lineedit->setInputMask("000.000.000.000");
    // FIXME: Take ownership or not?
    m_lineedit->setValidator(new IPv4AddressValidator);
    m_lineedit->setCursorPosition(0);

    auto layout = new QHBoxLayout(this);
    layout->setContentsMargins({});
    layout->addWidget(m_lineedit);

    connect(m_lineedit, &QLineEdit::editingFinished, this,
            &IPv4Edit::updateIpv4Address);
}

IPv4Edit::~IPv4Edit() {}

void IPv4Edit::setIpv4Address(const QString &ip)
{
    if (ip != m_ipv4Address) {
        m_ipv4Address = ip;
        emit ipAddressChanged();
    }
}

QString IPv4Edit::ipv4Address() const { return m_ipv4Address; }

bool IPv4Edit::isEditReallyFinished(const QString &ipv4Address) const
{
    QHostAddress addr;
    return addr.setAddress(ipv4Address);
}

void IPv4Edit::updateIpv4Address()
{
    QString ipv4Address{"999.999.999.999"};
    int index{0};
    for (const auto &octet : m_lineedit->text()) {
        if (octet.isDigit() || octet == '.') {
            ipv4Address[index] = octet;
            index++;
        }
    }

    if (isEditReallyFinished(ipv4Address)) {
        setIpv4Address(ipv4Address);
    }
}

} // namespace thoht

#include "IPv4LineEdit.moc"
#include "moc_IPv4LineEdit.cpp"
