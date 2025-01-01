#include "qtthread.h"

namespace tl {

SuspendableWorker::SuspendableWorker(QObject* parent) : QObject(parent)
{
    m_waitMutex.lock();
}

SuspendableWorker::~SuspendableWorker()
{
    m_waitCondition.wakeAll();
    m_waitMutex.unlock();
}

void SuspendableWorker::suspend()
{
    QMetaObject::invokeMethod(this, &SuspendableWorker::suspendImpl);
    m_waitMutex.lock();
    m_waitMutex.unlock();
}

void SuspendableWorker::resume() { m_waitCondition.wakeAll(); }

void SuspendableWorker::suspendImpl() { m_waitCondition.wait(&m_waitMutex); }

} // namespace tl

#include "moc_qtthread.cpp"
