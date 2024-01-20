#pragma once

#include <QWidget>
#include "../Alarms.h"

class QTableWidgetItem;

namespace tl {

namespace Ui {
class AlarmConfigurationWidget;
}

class AlarmConfigurationWidget : public QWidget
{
    Q_OBJECT

public:
    enum EditionMode
    {
        emCreation,
        emEdition,
        emElementConnected,
        emElementDisconnected,
        emElementStandAlone
    };

    explicit AlarmConfigurationWidget(QWidget *parent = nullptr);
    ~AlarmConfigurationWidget() override;

    void setEditionMode(const EditionMode &mode);

    void updateConfig();
    void updateUi();

    bool isConnected();
    QString getUiName();
    QExplicitlySharedDataPointer<Alarms> alarmConfig() const;
    void setAlarmConfig(
        const QExplicitlySharedDataPointer<Alarms> &alarmConfig);

public slots:
    void filter();
    void clearFilter();
    void newAlarm();
    void moveDown();
    void moveUp();
    void removeAlarm();
    void doubleClickedItem(QTableWidgetItem *item);

signals:
    void stateChanged(int state);

protected:
    void resizeEvent(QResizeEvent *event) override;
    void resizeColumns();

private:
    Ui::AlarmConfigurationWidget *ui;
    EditionMode mEditionMode;
    QExplicitlySharedDataPointer<Alarms> mAlarmConfig;
    bool mModified;
};

} // namespace tl
