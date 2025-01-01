#pragma once

#include <QLoggingCategory>

#include "gui/qtcoreutils.h"
#include "ToolView.h"

namespace tl {

Q_DECLARE_LOGGING_CATEGORY(StereoVerificationV2)

class StereoCameraVerificationViewPrivate;
class StereoCameraVerificationView : public ToolView
{
    Q_OBJECT

public:
    explicit StereoCameraVerificationView(QWidget* parent = nullptr);
    ~StereoCameraVerificationView();

    void setDevMode(bool enable) override;

    bool setFromJson(const std::string& json) override;

    void restoreSettings(QSettings& settings) override;
    void saveSettings(QSettings& settings) const override;

private:
    Q_DECLARE_PIMPL(StereoCameraVerificationView);
};

} // namespace tl
