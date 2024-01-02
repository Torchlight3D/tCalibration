#pragma once

#include "../BoardElement.h"

namespace thoht {

class AdaptiveTextElement;
class LayoutList;
class LayoutGrid;

class AlarmPanel : public BoardElement
{
    Q_OBJECT

public:
    explicit AlarmPanel(Board *dashboard = nullptr);

    // reimplemented virtual methods:
    int defaultWidth() override;
    int defaultHeight() override;
    void initialize(Board *dashboard) override;
    void clearElement() override;
    void resetElement() override;
    void buildElement() override;
    void edit() override;

    void update(UpdatePhase phase) override;
    void loadData() override;
    void displayData() override;
    void checkParameters() override;

    void saveSettings(QSettings *settings) override;
    void loadSettings(QSettings *settings) override;

    void saveConfigurations(QSettings *settings) override;
    void loadConfigurations(QSettings *settings) override;

    QSharedPointer<BoardParameter> addParameter(
        ParameterConfiguration::Ptr parameterSettings)
        override;
    QSharedPointer<BoardParameter> addParameter(QString paramLabel) override;
    QSharedPointer<BoardParameter> addParameter(
        QSharedPointer<TimeSeries> dataParameter) override;
    void removeBoardParameters() override;

    void addAlarm(QExplicitlySharedDataPointer<Alarms> alarmConfig) override;
    void updateAlarmConfiguration();

    bool connected() const;
    void disconnectAlarmConfig();
    void resetModifications() override;
    void beforeDataUpdate(bool refresh) override;

    void setRowHeight(int rowHeight);
    int rowHeight() const;
    void setAutoRowHeight(bool autoRowHeight);
    bool autoRowHeight() const;

    void setHeaderVisible(bool visible);
    bool headerVisible() const;
    void setAutoHeaderText(bool autoHeaderText);
    bool autoHeaderText() const;
    void setHeaderText(const QString &headerText);
    QString headerText() const;

protected:
    void draw(QCPPainter *painter) override;

    AdaptiveTextElement *mHeader;
    QList<AdaptiveTextElement *> mTextElements;
    LayoutGrid *mMainLayout{};
    LayoutList *mListLayout;

    QList<QSharedPointer<BoardParameter>> mDashParametersPrimary;
    QList<QSharedPointer<BoardParameter>> mDashParametersSecondary;

    bool mAlarmConfigurationIsShared;
    QExplicitlySharedDataPointer<Alarms> mSharedAlarmConfiguration;
    QExplicitlySharedDataPointer<Alarms> mExclusiveAlarmConfiguration;
    QExplicitlySharedDataPointer<Alarms> mAlarmConfiguration;

    bool mUpdatingConfig;
    bool mAutoRowHeight;
    int mRowHeight;
    bool mHeaderVisible;
    bool mAutoHeaderText;
    QString mHeaderText;

    QPen mListBorderPen;
};

} // namespace thoht
