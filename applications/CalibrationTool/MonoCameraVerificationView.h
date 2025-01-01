#pragma once

#include <QLoggingCategory>

#include "gui/qtcoreutils.h"
#include "ToolView.h"

namespace tl {

Q_DECLARE_LOGGING_CATEGORY(MonoCalibration)

class MonoCameraVerificationViewPrivate;
class MonoCameraVerificationView : public ToolView
{
    Q_OBJECT

public:
    explicit MonoCameraVerificationView(QWidget* parent = nullptr);
    ~MonoCameraVerificationView();

    bool setFromJson(const std::string& json) override;

    void restoreSettings(QSettings& settings) override;
    void saveSettings(QSettings& settings) const override;

private:
    Q_DECLARE_PIMPL(MonoCameraVerificationView);
};

} // namespace tl
