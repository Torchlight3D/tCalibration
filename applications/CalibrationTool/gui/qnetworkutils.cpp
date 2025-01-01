#include "qnetworkutils.h"

#include <QHostAddress>
#include <QProcess>
#include <QRegularExpression>

namespace tl {

using namespace Qt::Literals::StringLiterals;

namespace qnetwork {

QString findDeviceHardwareAddress(const QString &addr, int timeout)
{
    if (QHostAddress{addr}.isNull()) {
        return {};
    }

    QProcess proc;

    proc.start(u"arp"_s, {u"-a"_s});
    proc.waitForFinished();

    const auto arpOutput = proc.readAllStandardOutput();
    if (arpOutput.isEmpty()) {
        return {};
    }

    // Handle pipes in command. Not sure this is the best way.
#ifdef Q_OS_WIN
    proc.start(u"findstr"_s, {addr});
#elif
    proc.start(u"grep"_s, {ip});
#endif
    proc.write(arpOutput);
    proc.closeWriteChannel();
    proc.waitForFinished();

    const auto res = proc.readAllStandardOutput();
    if (res.isEmpty()) {
        return {};
    }

    static const QRegularExpression regex{
        R"(\b([0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2}\b)"};

    const auto matches =
        regex.globalMatch(res, 0, QRegularExpression::PartialPreferFirstMatch);
    if (!matches.isValid()) {
        return {};
    }

    return matches.peekNext().captured(0);
}

bool ping(const QString &addr, int count, int timeout)
{
    if (QHostAddress{addr}.isNull()) {
        return false;
    }

    QProcess proc;

    const auto program = u"ping"_s;
#ifdef Q_OS_WIN
    const auto arguments = QStringList{} << u"-n"_s << QString::number(count)
                                         << u"-w"_s << QString::number(timeout)
                                         << addr;
#else
    const auto arguments = QStringList{} << u"-c"_s << QString::number(count)
                                         << u"-w"_s << QString::number(timeout)
                                         << addr;
#endif

    proc.start(program, arguments);
    proc.waitForFinished();

    return proc.exitStatus() == QProcess::NormalExit && proc.exitCode() == 0;
}

} // namespace qnetwork
} // namespace tl
