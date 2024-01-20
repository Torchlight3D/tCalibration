#pragma once

#include <QThread>

#include "SuspendableWorker.h"

namespace tl {

template <typename Worker_t>
class AutoDeleteThread : public QThread
{
public:
    explicit AutoDeleteThread(Worker_t* worker, QObject* parent = nullptr)
        : QThread(parent), m_worker(worker)
    {
        m_worker->moveToThread(this);
        start();
    }

    ~AutoDeleteThread()
    {
        resume();
        quit();
        wait();
    }

    Worker_t* worker() const { return m_worker; }

    /// Actions
    void suspend()
    {
        if (auto worker = qobject_cast<SuspendableWorker*>(m_worker)) {
            worker->suspend();
        }
    }

    void resume()
    {
        if (auto worker = qobject_cast<SuspendableWorker*>(m_worker)) {
            worker->resume();
        }
    }

protected:
    void run() override
    {
        QThread::run();
        delete m_worker;
    }

private:
    Worker_t* m_worker; // Own
};

} // namespace tl
