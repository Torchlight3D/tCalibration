#pragma once

#include <QWidget>
#include "gui/guiutils.h"

namespace thoht {

class StereoModuleToolViewPrivate;
class StereoModuleToolView : public QWidget
{
    Q_OBJECT

public:
    explicit StereoModuleToolView(QWidget *parent = nullptr);
    ~StereoModuleToolView();

    enum Page
    {
        ImageQuality,
        Calibration,
        Verification,
    };
    void setHomePage(Page home);

    void setDevMode(bool on);
    bool isDevMode() const;

private:
    Q_DECLARE_PIMPL(StereoModuleToolView)
};

} // namespace thoht
