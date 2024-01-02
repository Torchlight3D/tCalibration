#pragma once

#include <QDialog>
#include <QWidget>

#include "gui/guiutils.h"
#include <AxCalib/StereoCameraVerification>

namespace thoht {

class StereoModuleVerificationConfigWidgetPrivate;
class StereoModuleVerificationConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StereoModuleVerificationConfigWidget(QWidget* parent = nullptr);
    ~StereoModuleVerificationConfigWidget();

    void setDefaultValues(const StereoCameraVerification::Options& options);
    StereoCameraVerification::Options options() const;

private:
    Q_DECLARE_PIMPL(StereoModuleVerificationConfigWidget)
};

// A Dialog wrapper
class StereoModuleVerificationConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit StereoModuleVerificationConfigDialog(QWidget* parent = nullptr);

    // Don't take ownership
    StereoModuleVerificationConfigWidget* const widget() const;

private:
    StereoModuleVerificationConfigWidget* const w;
};

} // namespace thoht
