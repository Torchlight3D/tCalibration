#pragma once

#include <QDialog>
#include <QWidget>

#include "gui/guiutils.h"
#include "StereoModuleTask.h"

namespace thoht {

class StereoModuleCalibrationConfigWidgetPrivate;
class StereoModuleCalibrationConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StereoModuleCalibrationConfigWidget(QWidget* parent = nullptr);
    ~StereoModuleCalibrationConfigWidget();

    void setDefaultValues(const StereoModuleTask::ResultReference& ref);

    StereoModuleTask::Options options() const;
    StereoModuleTask::ResultReference resultReference() const;

private:
    Q_DECLARE_PIMPL(StereoModuleCalibrationConfigWidget)
};

// A Dialog wrapper
class StereoModuleCalibrationConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit StereoModuleCalibrationConfigDialog(QWidget* parent = nullptr);

    // Don't take ownership
    StereoModuleCalibrationConfigWidget* const widget() const;

private:
    StereoModuleCalibrationConfigWidget* const w;
};

} // namespace thoht
