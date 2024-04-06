﻿#include <glog/logging.h>

#include <filesystem>

#include <QApplication>
#include <QDir>
#include <QLocale>
#include <QProcess>
#include <QThreadPool>
#include <QTranslator>

#include "gui/TheStyle.h"
#include "gui/qstringutils.h"
#include "AppConfigs.h"

namespace fs = std::filesystem;

void setupGlog(const char *name)
{
    constexpr char kLogDir[]{"Log"};

    const fs::path logDirPath{kLogDir};
    if (!fs::exists(logDirPath)) {
        fs::create_directories(logDirPath);
    }

    //    FLAGS_logtostdout = true;
    //    FLAGS_logtostderr = true;
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = kLogDir;
    ::google::InitGoogleLogging(name);
}

void setupLanguage(QApplication &app)
{
    const std::vector locales{
        QLocale{QLocale::Language::Chinese, QLocale::Territory::China}};
    for (const auto &locale : locales) {
        for (const auto &qmFile : {u"qtbase"_s, u"CalibrationTool"_s}) {
            auto translator = new QTranslator(&app);
            if (translator->load(locale, qmFile, "_")) {
                app.installTranslator(translator);
            }
        }
    }
}

void setupStyle(QApplication &app)
{
    auto *const style = new tl::QlementineStyle(&app);
    style->setAnimationsEnabled(true);
    style->setUseMenuForComboBoxPopup(false);
    style->setAutoIconColorEnabled(true);
    style->setThemeJsonPath(u":themes/light.json"_s);
    app.setStyle(style);
}

bool invokeCameraRouter(QApplication &app)
{
    const auto cameraRouterPath = QDir::home().absoluteFilePath("CameraRouter");
    if (!QFileInfo::exists(cameraRouterPath)) {
        qCritical() << "Failed to open CameraRouter: " << cameraRouterPath
                    << " executable not exists.";
        return false;
    }

    // Leave it here, don't care its life time
    auto *cameraRouter = new QProcess(&app);
    cameraRouter->start(cameraRouterPath);
    return true;
}

int main(int argc, char *argv[])
{
    setupGlog(argv[0]);

    // Handle high dpi screen, FIXME: greatly increase memory
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
#ifdef Q_OS_WIN
    // Workaround crash bug #408 in Qt/Windows
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
        Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
#endif

    constexpr char kConfigsPath[]{"configs.tt"};

    tl::AppConfigs configs;
    [[maybe_unused]] const bool configsLoaded =
        tl::AppConfigs::load(kConfigsPath, configs);

    QApplication app{argc, argv};

    setupLanguage(app);

    if (configs.useCustomStyle) {
        setupStyle(app);
    }

    if (configs.invokeCameraRouter && !invokeCameraRouter(app)) {
        return 0;
    }

    // Wait for the threads opened by QtConcurrent(default global QThreadPool)
    // to finish, or the app would close with crashes.
    // QUEST: Not sure this is the best solution?
    QObject::connect(qApp, &QCoreApplication::aboutToQuit, qApp, []() {
        QThreadPool::globalInstance()->waitForDone(2000);
    });

    tl::StereoModuleToolView w;
    w.resize(1000, 600);
    w.setHomePage(configs.homePage);
    w.show();

    return app.exec();
}
