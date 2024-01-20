#pragma once

#include "StereoModuleToolView.h"

namespace tl {

struct AppConfigs
{
    StereoModuleToolView::Page homePage =
        StereoModuleToolView::Page::Calibration;
    bool useCustomStyle = false;
    bool invokeCameraRouter = false;

    static bool load(const QString& filename, AppConfigs& configs);
    static bool load(const QJsonObject& jo, AppConfigs& configs);
};

} // namespace tl
