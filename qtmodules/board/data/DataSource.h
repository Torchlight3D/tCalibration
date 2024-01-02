#pragma once

#include <QObject>

namespace thoht {

class DataManager;
class TimeSeries;

class DataSource : public QObject
{
    Q_OBJECT

public:
    enum DataSourceStatus
    {
        dssIdle,
        dssRunning,
        dssError
    };
    Q_ENUM(DataSourceStatus)

    DataSource(QString name) : QObject(), mStatus(dssIdle), mName(name) {}

    virtual ~DataSource() {}

    virtual bool registerParameter(const QSharedPointer<TimeSeries> &param) = 0;
    virtual void unregisterParameter(quint32 parameterId) = 0;
    virtual void unregisterParameter(const QString &label) = 0;
    virtual void clearParameters() = 0;

    QString name() const { return mName; }

signals:
    void statusChanged();

protected:
    void setDataManager(DataManager *dataManager)
    {
        if (dataManager) {
            mDataManager = dataManager;
            dataManagerInitialized();
        }
    }

    virtual void dataManagerInitialized() {}

    void setStatus(const DataSourceStatus &status)
    {
        if (status != mStatus) {
            mStatus = status;
            emit statusChanged();
        }
    }

    DataSourceStatus status() { return mStatus; }

    DataManager *mDataManager;
    DataSourceStatus mStatus;
    QVector<QSharedPointer<TimeSeries>> mListParam;
    QString mName;

    friend class DataManager;
    friend class SettingsDialog;
};

} // namespace thoht
