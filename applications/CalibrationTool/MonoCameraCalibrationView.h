#pragma once

#include <QLoggingCategory>

#include "gui/qtcoreutils.h"
#include "ToolView.h"

namespace tl {

Q_DECLARE_LOGGING_CATEGORY(MonoCalibration)

// FIXME:
// 1. Most of the codes are copied from StereoCameraCalibrationView, is it
// possible to abstract the pattern?
class MonoCameraCalibrationViewPrivate;
class MonoCameraCalibrationView : public ToolView
{
    Q_OBJECT

public:
    explicit MonoCameraCalibrationView(QWidget* parent = nullptr);
    ~MonoCameraCalibrationView();

    /// Properties
    void setDevMode(bool enable) override;

    bool setFromJson(const std::string& json) override;

    void restoreSettings(QSettings& settings) override;
    void saveSettings(QSettings& settings) const override;

private:
    Q_DECLARE_PIMPL(MonoCameraCalibrationView);
};

} // namespace tl
