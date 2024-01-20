#pragma once

#include <QColor>
#include <QSharedData>

class QSettings;

namespace tl {

class Alarms : public QSharedData
{
public:
    enum AlarmFunction
    {
        afGreaterThan,
        afLowerThan,
        afBitSet,
        afParamDead
    };

    Alarms();

    void saveToFile(const QString &saveDirectory);
    void loadFromFile(const QString &settingsFile);

    void save(QSettings *settings);
    void load(QSettings *settings);

    void setName(const QString &name);
    QString name() const;

    int alarmsCount() { return mActive.count(); }
    void clearAlarms();

    void addAlarm(bool active, QString watchedParam, QString message,
                  QString mOutputParam, AlarmFunction function, double arg,
                  QColor color);

    QList<bool> active() const;
    QStringList watchedParameters() const;
    QStringList messages() const;
    QStringList outputParameters() const;
    QList<AlarmFunction> functions() const;
    QList<double> functionArgs() const;
    QList<QColor> colors() const;

    void setModified(bool modified);
    bool modified() const;

    QString saveFilePath() const;

protected:
    QString mSaveFilePath;
    QString mName;
    QList<bool> mActive;
    QStringList mWatchedParameters;
    QStringList mMessages;
    QStringList mOutputParameters;
    QList<AlarmFunction> mFunctions;
    QList<double> mFunctionArgs;
    QList<QColor> mColors;
    bool mModified;
};

} // namespace tl
