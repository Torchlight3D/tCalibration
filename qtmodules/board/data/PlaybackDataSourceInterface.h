﻿#pragma once

#include "DataSource.h"
#include "DataManager.h"
#include "TimeSeries.h"

namespace tl {

class PlaybackDataSourceInterface : public DataSource
{
    Q_OBJECT

public:
    PlaybackDataSourceInterface(QString name) : DataSource(name) {}
    virtual ~PlaybackDataSourceInterface() override {}

    virtual bool activatePlayback() = 0;
    virtual void setPlaybackDateTime(double secsSinceStartOfTheDay,
                                     QDate date = QDate()) = 0;
    virtual void loadPlaybackData(
        QVector<QSharedPointer<TimeSeries>> boardParameters,
        QMap<quint32, QVector<double>>& timestamps,
        QMap<quint32, QVector<DataValue>>& values) = 0;

    virtual bool parametersUpdated() = 0;
    void clearParameters() override
    {
        if (mDataManager) {
            foreach (auto param, mListParam)
                mDataManager->unregisterPlaybackParameter(param);
            mListParam.clear();
        }
    }

    void unregisterParameter(const QString& label) override
    {
        if (mDataManager) {
            QSharedPointer<TimeSeries> param;
            param = mDataManager->playbackParameter(label);
            mDataManager->unregisterPlaybackParameter(label);
            if (param)
                mListParam.removeAll(param);
        }
    }

    void unregisterParameter(quint32 parameterId) override
    {
        if (mDataManager) {
            QSharedPointer<TimeSeries> param;
            param = mDataManager->playbackParameter(parameterId);
            mDataManager->unregisterPlaybackParameter(parameterId);
            if (param)
                mListParam.removeAll(param);
        }
    }

    virtual bool registerParameter(
        const QSharedPointer<TimeSeries>& param) override
    {
        bool st = false;
        if (mDataManager) {
            if (param->valid()) {
                st = mDataManager->registerPlaybackParameter(param);
                if (st)
                    mListParam.append(param);
            }
            else {
                //                logger()->debug()
                //                    << Q_FUNC_INFO
                //                    << "Param is not valid - failed to
                //                    allocate memory"
                //                    << param->name() << param->capacity();
            }
        }

        return st;
    }

signals:
    void playbackStartTime(int);
    void playbackEndTime(int);

protected:
    virtual void dataManagerInitialized() override
    {
        connect(this, &PlaybackDataSourceInterface::playbackStartTime,
                mDataManager, &DataManager::newPlaybackStartTime);
        connect(this, &PlaybackDataSourceInterface::playbackEndTime,
                mDataManager, &DataManager::newPlaybackEndTime);
    }
};

} // namespace tl

#define PlaybackDataSourceInterface_iid "tl.PlaybackDataSources"

Q_DECLARE_INTERFACE(tl::PlaybackDataSourceInterface,
                    PlaybackDataSourceInterface_iid)
