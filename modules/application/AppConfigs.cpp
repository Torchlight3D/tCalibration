#include "AppConfigs.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

namespace thoht {

namespace key {
constexpr char kAppConfigs[]{"app"};
constexpr char kHomePage[]{"home_page"};
constexpr char kUseCustomStyle[]{"use_custom_style"};
constexpr char kInvokeCameraRouter[]{"invoke_camera_router"};
} // namespace key

bool AppConfigs::load(const QString& filename, AppConfigs& configs)
{
    QFile file{filename};
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Failed to open app configs file. " << filename;
        return false;
    }

    const auto jDoc = QJsonDocument::fromJson(file.readAll());
    if (!jDoc.isObject()) {
        qWarning() << "Invalid JSON document format.";
        return false;
    }

    const auto joAll = jDoc.object();
    if (auto jvApp = joAll.value(key::kAppConfigs); jvApp.isUndefined()) {
        return false;
    }
    else {
        return AppConfigs::load(jvApp.toObject(), configs);
    }
}

bool AppConfigs::load(const QJsonObject& jo, AppConfigs& configs)
{
    if (jo.isEmpty()) {
        qWarning() << "Empty app configs JSON format.";
        return false;
    }

    if (auto jvHomePage = jo.value(key::kHomePage); !jvHomePage.isUndefined()) {
        configs.homePage = StereoModuleToolView::Page(jvHomePage.toInt());
    }
    if (auto jvUseCustomType = jo.value(key::kUseCustomStyle);
        !jvUseCustomType.isUndefined()) {
        configs.useCustomStyle = jvUseCustomType.toBool(false);
    }
    if (auto jvInvokeCameraRouter = jo.value(key::kInvokeCameraRouter);
        !jvInvokeCameraRouter.isUndefined()) {
        configs.invokeCameraRouter = jvInvokeCameraRouter.toBool(false);
    }

    return true;
}

} // namespace thoht
