#pragma once

#include <QDebug>
#include <QDir>
#include <QDateTime>
#include <QFileSystemWatcher>
#include <QObject>
#include <QSettings>

#include "settings/ParameterConfiguration.h"

#define PRO_SETTINGS_KEY_NAME "Name"
#define PRO_SETTINGS_KEY_CREATIONDATE "CreationDate"
#define PRO_SETTINGS_KEY_MODIFICATIONDATE "ModificationDate"

namespace thoht {

struct ElementDecl
{
    QString page_name;
    QString parameter;
    QString type_name;
    int column;
    int row;
};

class Alarms;
class Page;

class Project : public QObject
{
    Q_OBJECT

public:
    explicit Project(QObject* parent = nullptr);
    ~Project();

    bool generate(const QString& workingDirectory, const QString& projectName);
    bool load(const QString& workingDirectory, const QString& projectName,
              bool fullLoad = true);
    void clear();

    void fullReload();

    void loadPages();
    bool addPage(QExplicitlySharedDataPointer<Page> page);
    QExplicitlySharedDataPointer<Page> page(const QString& pageName);
    QExplicitlySharedDataPointer<Page> currentPage();
    QExplicitlySharedDataPointer<Page> requestedPage();
    QMap<QString, QExplicitlySharedDataPointer<Page>> pages() const;

    bool exportToCsv(const QString& filePath);

    void loadParametersSettings();
    void addParameterSettings(ParameterConfiguration::Ptr parameterSettings);
    ParameterConfiguration::Ptr parameterSettings(const QString& name,
                                                  const QString& description);
    QMap<QString, QMap<QString, ParameterConfiguration::Ptr>>
    parametersSettings() const;

    void loadAlarmsConfigurations();
    void addAlarmConfiguration(
        QExplicitlySharedDataPointer<Alarms> alarmConfig);
    QMap<QString, QExplicitlySharedDataPointer<Alarms>> alarmsConfigurations()
        const;
    QExplicitlySharedDataPointer<Alarms> alarmConfiguration(
        const QString& name);

    void setName(const QString& name);
    QString name() const;

    void setCreationDate(const QDateTime& creationDate);
    QDateTime creationDate() const;

    void setModificationDate(const QDateTime& modificationDate);
    QDateTime modificationDate() const;

    void setCurrentPageName(const QString& currentPageName);
    QString currentPageName() const;

    void setRequestedPageName(const QString& requestedPageName);
    QString requestedPageName() const;

    QList<ElementDecl> findParameter(const QString& parameterName);

    QString alarmsConfigPath() const;
    QString pagesPath() const;
    QString paramSettingsPath() const;
    QString resourcesImagesPath() const;
    QString resourcesPath() const;

public slots:
    void requestPage(const QString& pageName);
    //    void alarmFilesChanged(QString path);

signals:
    void loaded();
    void pagesListUpdated();
    void pageRequested();
    void pageLoaded();
    void updated();

private:
    QDateTime mCreationDate;
    QDateTime mModificationDate;
    QString mName;
    QString mProjectPath;
    QString mSettingsPath;

    QString mCurrentPageName;
    QString mRequestedPageName;

    QString mResourcesPath;
    QString mResourcesImagesPath;

    QString mPagesPath;
    bool mPagesDirWrittable;

    QString mAlarmsConfigPath;
    QString mParamSettingsPath;
    bool mParamSettingsDirWrittable;

    QMap<QString, QExplicitlySharedDataPointer<Page>> mPages;
    QMap<QString, QMap<QString, ParameterConfiguration::Ptr>>
        mParametersSettings;
    QMap<QString, QExplicitlySharedDataPointer<Alarms>> mAlarmsConfigurations;

    //    QFileSystemWatcher *mAlarmsWatcher;
};

} // namespace thoht
