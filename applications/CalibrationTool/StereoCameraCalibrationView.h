#pragma once

#include <QLoggingCategory>

#include "gui/qtcoreutils.h"
#include "ToolView.h"

namespace tl {

Q_DECLARE_LOGGING_CATEGORY(StereoCalibrationV2)

class StereoCameraCalibrationViewPrivate;
class StereoCameraCalibrationView : public ToolView
{
    Q_OBJECT

public:
    explicit StereoCameraCalibrationView(QWidget* parent = nullptr);
    ~StereoCameraCalibrationView();

    /// Properties
    void setDevMode(bool enable) override;

    bool setFromJson(const std::string& json) override;

    void restoreSettings(QSettings& settings) override;
    void saveSettings(QSettings& settings) const override;

private:
    Q_DECLARE_PIMPL(StereoCameraCalibrationView);
};

} // namespace tl
