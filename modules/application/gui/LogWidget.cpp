#include "LogWidget.h"

#ifdef Q_OS_WIN
#include <io.h>
#include <fcntl.h>
#else
#include <unistd.h>
#include <sys/ioctl.h>
#endif

#include <QGridLayout>
#include <QMutex>
#include <QMutexLocker>
#include <QPlainTextEdit>
#include <QPointer>
#include <QPushButton>
#include <QSocketNotifier>
#include <QTimer>
#include <QUuid>
#ifdef Q_OS_WIN
#include <qt_windows.h>
#include <private/qwindowspipereader_p.h>
#else
#include <private/qringbuffer_p.h>
#endif

#include "guiutils.h"

namespace thoht {

///------- StdoutRedirector starts from here
class StdoutRedirector : public QObject
{
    Q_OBJECT

public:
    enum ProcessChannel
    {
        StandardOutput = 1,
        StandardError = 2
    };
    Q_DECLARE_FLAGS(ProcessChannels, ProcessChannel)

    explicit StdoutRedirector(QObject* parent = nullptr,
                              ProcessChannels channels = StandardOutput);
    ~StdoutRedirector();

    qint64 bytesAvailable() const;
    QByteArray read(qint64 maxlen);

signals:
    void readyRead(const QByteArray& msg);

private slots:
    void onSocketActivated();

private:
    ProcessChannels m_channels;
#ifdef Q_OS_WIN
    HANDLE hRead;
    HANDLE hWrite;
    QWindowsPipeReader* pipeReader;
#else
    int pipeEnds[2];
    QRingBuffer* buffer;
    QSocketNotifier* socketNotifier;
#endif
};

Q_DECLARE_OPERATORS_FOR_FLAGS(StdoutRedirector::ProcessChannels)

#ifdef Q_OS_WIN
namespace {
void createWinPipe(HANDLE& hRead, HANDLE& hWrite)
{
    const auto pipeName =
        QString::fromLatin1("\\\\.\\pipe\\stdoutredirector-%1")
            .arg(QUuid::createUuid().toString());
    SECURITY_ATTRIBUTES attributes = {sizeof(SECURITY_ATTRIBUTES), 0, false};
    hRead = ::CreateNamedPipe(
        (wchar_t*)pipeName.utf16(), PIPE_ACCESS_INBOUND | FILE_FLAG_OVERLAPPED,
        PIPE_TYPE_BYTE | PIPE_WAIT, 1, 0, 1024 * 1024, 0, &attributes);

    SECURITY_ATTRIBUTES attributes2 = {sizeof(SECURITY_ATTRIBUTES), 0, true};
    hWrite =
        ::CreateFile((wchar_t*)pipeName.utf16(), GENERIC_WRITE, 0, &attributes2,
                     OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);

    ::ConnectNamedPipe(hRead, NULL);
}
} // namespace
#endif

StdoutRedirector::StdoutRedirector(QObject* parent, ProcessChannels channels)
    : QObject(parent), m_channels(channels)
{
#ifdef Q_OS_WIN
    createWinPipe(hRead, hWrite);
    if (m_channels & StandardOutput)
        ::SetStdHandle(STD_OUTPUT_HANDLE, hWrite);
    if (m_channels & StandardError)
        ::SetStdHandle(STD_ERROR_HANDLE, hWrite);

    int fd = _open_osfhandle((intptr_t)hWrite, _O_WRONLY | _O_TEXT);
    if (m_channels & StandardOutput)
        _dup2(fd, 1);
    if (m_channels & StandardError)
        _dup2(fd, 2);
    _close(fd);

    pipeReader = new QWindowsPipeReader(this);
    pipeReader->setHandle(hRead);
    pipeReader->startAsyncRead();
    connect(pipeReader, SIGNAL(readyRead()), this, SIGNAL(readyRead()));
#else
    [[maybe_unused]] const auto _ = ::pipe(pipeEnds);
    if (m_channels & StandardOutput)
        ::dup2(pipeEnds[1], 1);
    if (m_channels & StandardError)
        ::dup2(pipeEnds[1], 2);
    ::close(pipeEnds[1]);

    buffer = new QRingBuffer();
    socketNotifier =
        new QSocketNotifier(pipeEnds[0], QSocketNotifier::Read, this);
    connect(socketNotifier, &QSocketNotifier::activated, this,
            &StdoutRedirector::onSocketActivated);
#endif
}

StdoutRedirector::~StdoutRedirector()
{
#ifdef Q_OS_WIN
    pipeReader->stop();
    ::DisconnectNamedPipe(hRead);
//    ::CloseHandle(hWrite);
#else
    delete buffer;
#endif
}

qint64 StdoutRedirector::bytesAvailable() const
{
#ifdef Q_OS_WIN
    return pipeReader->bytesAvailable();
#else
    return buffer->size();
#endif
}

QByteArray StdoutRedirector::read(qint64 maxlen)
{
#ifdef Q_OS_WIN
    QByteArray result(int(maxlen), Qt::Uninitialized);
    qint64 readBytes = pipeReader->read(result.data(), result.size());
    if (readBytes <= 0)
        result.clear();
    else
        result.resize(int(readBytes));
    return result;
#else
    return buffer->read();
#endif
}

void StdoutRedirector::onSocketActivated()
{
#ifdef Q_OS_UNIX
    int bytesQueued;
    if (::ioctl(pipeEnds[0], FIONREAD, &bytesQueued) == -1)
        return;
    if (bytesQueued <= 0)
        return;
    char* writePtr = buffer->reserve(bytesQueued);
    int bytesRead = ::read(pipeEnds[0], writePtr, bytesQueued);
    if (bytesRead < bytesQueued)
        buffer->chop(bytesQueued - bytesRead);
    emit readyRead(read(1024));
#endif
}

///------- LogWidgetPrivate starts from here
class LogWidgetPrivate
{
    Q_DEFINE_PIMPL(LogWidget)

public:
    explicit LogWidgetPrivate(LogWidget* q);
    ~LogWidgetPrivate();

    void init();

    void append(const std::string& message);

public:
    QMutex m_mtx;
    std::string m_msgQueue;
    // TODO: Use TextBrowser
    QWidget* m_toolBar;
    QPlainTextEdit* m_msgBrowser;
    QPointer<StdoutRedirector> m_stdoutRedirector;
};

LogWidgetPrivate::LogWidgetPrivate(LogWidget* q) : q_ptr(q) {}

LogWidgetPrivate::~LogWidgetPrivate()
{
    // TODO: Maybe handle multithread things
}

void LogWidgetPrivate::init()
{
    // Do nothing
}

void LogWidgetPrivate::append(const std::string& msg)
{
    QMutexLocker locker{&m_mtx};
    m_msgQueue += msg;
}

///------- LogWidget starts from here
LogWidget::LogWidget(QWidget* parent)
    : QWidget(parent), d_ptr(new LogWidgetPrivate(this))
{
    Q_D(LogWidget);
    d->init();

    setWindowTitle(tr("Log"));

    constexpr int kFlushInterval{100}; // ms
    auto* flushTimer = new QTimer(this);
    connect(flushTimer, &QTimer::timeout, this, &LogWidget::flush);
    flushTimer->start(kFlushInterval);

    // NOTE: Recommend to turn this off under debug mode, otherwise debug
    // messages won't appear on the console or this widget, and we can't trace
    // the errors.
    d->m_stdoutRedirector =
        new StdoutRedirector(this, StdoutRedirector::StandardOutput |
                                       StdoutRedirector::StandardError);
    connect(d->m_stdoutRedirector, &StdoutRedirector::readyRead, this,
            &LogWidget::append);

    auto clearBtn = new QPushButton(tr("Clear"), this);
    connect(clearBtn, &QPushButton::clicked, this, &LogWidget::clear);

    d->m_toolBar = new QWidget(this);
    auto* toolsLayout = new QHBoxLayout(d->m_toolBar);
    toolsLayout->setContentsMargins({});
    toolsLayout->setSpacing(6);
    toolsLayout->addWidget(clearBtn);
    toolsLayout->addStretch();

    constexpr int kDefaultMaxBlockCount{100000};
    d->m_msgBrowser = new QPlainTextEdit(this);
    d->m_msgBrowser->setReadOnly(true);
    d->m_msgBrowser->setMaximumBlockCount(kDefaultMaxBlockCount);
    d->m_msgBrowser->setWordWrapMode(QTextOption::NoWrap);
    d->m_msgBrowser->setFont(QFont("Courier", 10));

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins({});
    mainLayout->addWidget(d->m_toolBar);
    mainLayout->addWidget(d->m_msgBrowser);
}

LogWidget::~LogWidget() = default;

void LogWidget::setMaxBlockCount(int count)
{
    Q_D(LogWidget);
    d->m_msgBrowser->setMaximumBlockCount(count);
}

void LogWidget::setToolBarVisible(bool visible)
{
    Q_D(LogWidget);
    d->m_toolBar->setVisible(visible);
}

void LogWidget::flush()
{
    Q_D(LogWidget);
    if (d->m_msgQueue.empty()) {
        return;
    }

    QMutexLocker locker{&d->m_mtx};
    d->m_msgBrowser->moveCursor(QTextCursor::End);
    d->m_msgBrowser->insertPlainText(QString::fromStdString(d->m_msgQueue));
    d->m_msgBrowser->moveCursor(QTextCursor::End);
    d->m_msgQueue.clear();
}

void LogWidget::append(const QByteArray& msg)
{
    Q_D(LogWidget);
    d->append(msg.toStdString());
}

void LogWidget::clear()
{
    Q_D(LogWidget);
    QMutexLocker locker{&d->m_mtx};
    d->m_msgQueue.clear();
    d->m_msgBrowser->clear();
}

} // namespace thoht

#include "LogWidget.moc"
#include "moc_LogWidget.cpp"
