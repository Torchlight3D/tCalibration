#pragma once

#include <QWidget>
#include "gui/guiutils.h"

namespace thoht {

class StereoModuleImageQualityViewPrivate;
class StereoModuleImageQualityView : public QWidget
{
    Q_OBJECT

public:
    explicit StereoModuleImageQualityView(QWidget *parent = nullptr);
    ~StereoModuleImageQualityView();

    void setDevMode(bool on);

private:
    Q_DECLARE_PIMPL(StereoModuleImageQualityView)
};

} // namespace thoht
