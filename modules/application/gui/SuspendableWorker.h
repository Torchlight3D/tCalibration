#pragma once

#include <QMutex>
#include <QObject>
#include <QWaitCondition>

namespace tl {

class SuspendableWorker : public QObject
{
    Q_OBJECT

public:
    explicit SuspendableWorker(QObject* parent = nullptr);
    ~SuspendableWorker();

    // NOTE: suspend() must be called from the outer thread
    void suspend();

    // NOTE: resume() must be called from the outer thread
    void resume();

private slots:
    void suspendImpl();

private:
    QMutex m_waitMutex;
    QWaitCondition m_waitCondition;
};

} // namespace tl