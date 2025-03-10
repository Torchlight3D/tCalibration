﻿#pragma once

#include <QWidget>
#include "../settings/ParameterConfiguration.h"

namespace tl {

class Project;

namespace Ui {
class ElementPropertiesWidget;
}

class ElementPropertiesWidget : public QWidget
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

    explicit ElementPropertiesWidget(QWidget *parent = nullptr);
    ~ElementPropertiesWidget();

    bool isConnected();
    void updateUi(
        ParameterConfiguration::Ptr parameterSettings);
    void updateParameterSettings(
        ParameterConfiguration::Ptr parameterSettings);
    void setEditionMode(const EditionMode &mode);
    void setPropertiesMode(
        const ParameterConfiguration::ConfigurationMode &propertiesMode);
    void setProject(QSharedPointer<Project> project);
    ParameterConfiguration::Ptr currentSettings()
        const;

private slots:
    void createSharedConfiguration();
    void on_connectedCheckBox_stateChanged(int arg1);

    void on_colorModeComboBox_currentIndexChanged(int index);
    void on_defaultColorToolButton_clicked();
    void on_defaultColorToolButton_2_clicked();
    void on_listWidget_currentRowChanged(int currentRow);

    void on_validRangeCheckBox_stateChanged(int arg1);
    void on_outOfRangeColorModeComboBox_currentIndexChanged(int index);
    void on_outOfRangeCheckBox_stateChanged(int arg1);

    void removeHighThreshold_clicked();
    void removeLowThreshold_clicked();

    void removeState_clicked();
    void on_itemsColorComboBox_currentIndexChanged(int index);
    void on_itemsColorToolButton_clicked();

    void on_statesTableWidget_cellDoubleClicked(int row, int column);

    void on_editPropPushButton_clicked();
    void on_addStateToolButton_clicked();

    void on_addHighThresholdToolButton_clicked();
    void on_addLowThresholdToolButton_clicked();
    void on_highThresholdsTableWidget_cellDoubleClicked(int row, int column);

    void on_lowThresholdsTableWidget_cellDoubleClicked(int row, int column);

signals:
    void connectProperties(bool);

private:
    Ui::ElementPropertiesWidget *ui;
    QSharedPointer<Project> mProject;
    EditionMode mEditionMode;
    ParameterConfiguration::ConfigurationMode mPropertiesMode;
    bool mStatesModified;
    bool mThresholdsModified;
    bool mBitfieldsModified;
    ParameterConfiguration::Ptr mCurrentSettings;
};

} // namespace tl
