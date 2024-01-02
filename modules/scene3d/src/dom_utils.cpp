#include "dom_utils.h"

namespace {
inline void warning(const QString &message)
{
    qWarning("%s", message.toLatin1().constData());
}
} // namespace

namespace io {

qreal qrealFromDom(const QDomElement &e, const QString &attribute,
                   qreal defValue)
{
    qreal value = defValue;
    if (e.hasAttribute(attribute)) {
        const QString s = e.attribute(attribute);
        bool ok;
        value = s.toDouble(&ok);
        if (!ok) {
            warning(
                QString("'%1' is not a valid qreal syntax for attribute \"%2\" "
                        "in initialization of \"%3\". Setting value to %4.")
                    .arg(s, attribute, e.tagName(), QString::number(defValue)));
            value = defValue;
        }
    }
    else {
        warning(QString("\"%1\" attribute missing in initialization of \"%2\". "
                        "Setting value to %3.")
                    .arg(attribute, e.tagName(), QString::number(value)));
    }

#if defined(isnan)
    // The "isnan" method may not be available on all platforms.
    // Find its equivalent or simply remove these two lines
    if (isnan(value))
        warning(QString("Warning, attribute \"%1\" initialized to Not a "
                        "Number in \"%2\"")
                    .arg(attribute, e.tagName()));
#endif

    return value;
}

int intFromDom(const QDomElement &e, const QString &attribute, int defValue)
{
    int value = defValue;
    if (e.hasAttribute(attribute)) {
        const QString s = e.attribute(attribute);
        bool ok;
        value = s.toInt(&ok);
        if (!ok) {
            warning(
                QString("'%1' is not a valid integer syntax for attribute "
                        "\"%2\" "
                        "in initialization of \"%3\". Setting value to %4.")
                    .arg(s, attribute, e.tagName(), QString::number(defValue)));
            value = defValue;
        }
    }
    else {
        warning(QString("\"%1\" attribute missing in initialization of \"%2\". "
                        "Setting value to %3.")
                    .arg(attribute, e.tagName(), QString::number(value)));
    }

    return value;
}

unsigned int uintFromDom(const QDomElement &e, const QString &attribute,
                         unsigned int defValue)
{
    unsigned int value = defValue;
    if (e.hasAttribute(attribute)) {
        const QString s = e.attribute(attribute);
        bool ok;
        value = s.toUInt(&ok);
        if (!ok) {
            warning(
                QString("'%1' is not a valid unsigned integer syntax "
                        "for attribute "
                        "\"%2\" in initialization of \"%3\". Setting "
                        "value to %4.")
                    .arg(s, attribute, e.tagName(), QString::number(defValue)));
            value = defValue;
        }
    }
    else {
        warning(QString("\"%1\" attribute missing in initialization of \"%2\". "
                        "Setting value to %3.")
                    .arg(attribute, e.tagName(), QString::number(value)));
    }

    return value;
}

bool boolFromDom(const QDomElement &e, const QString &attribute, bool defValue)
{
    bool value = defValue;
    if (e.hasAttribute(attribute)) {
        const QString s = e.attribute(attribute);
        if (s.toLower() == QString("true"))
            value = true;
        else if (s.toLower() == QString("false"))
            value = false;
        else {
            warning(QString("'%1' is not a valid boolean syntax for attribute "
                            "\"%2\" "
                            "in initialization of \"%3\". Setting value to %4.")
                        .arg(s, attribute, e.tagName(),
                             defValue ? "true" : "false"));
        }
    }
    else {
        warning(QString("\"%1\" attribute missing in initialization of \"%2\". "
                        "Setting value to %3.")
                    .arg(attribute, e.tagName(), value ? "true" : "false"));
    }

    return value;
}

void setBoolAttribute(QDomElement &element, const QString &attribute,
                      bool value)
{
    element.setAttribute(attribute, (value ? "true" : "false"));
}

namespace key {
constexpr char kRed[]{"red"};
constexpr char kGreen[]{"green"};
constexpr char kBlue[]{"blue"};
} // namespace key

QDomElement QColorDomElement(const QColor &color, const QString &name,
                             QDomDocument &doc)
{
    auto de = doc.createElement(name);
    de.setAttribute(key::kRed, QString::number(color.red()));
    de.setAttribute(key::kGreen, QString::number(color.green()));
    de.setAttribute(key::kBlue, QString::number(color.blue()));
    return de;
}

QColor QColorFromDom(const QDomElement &e)
{
    int color[3]{0, 0, 0};
    const QStringList attribute{key::kRed, key::kGreen, key::kBlue};
    for (qsizetype i{0}; i < attribute.count(); ++i) {
        color[i] = intFromDom(e, attribute[i], 0);
    }

    return {color[0], color[1], color[2]};
}

} // namespace io
