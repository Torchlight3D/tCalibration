#pragma once

#include <QWidget>

#include "gui/qtcoreutils.h"

namespace tl {

class MainWindowPrivate;
class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    enum Page
    {
        MonoCalibration,
        MonoVerification,
        StereoCalibrationV2,
        StereoVerificationV2,
        StereoCalibrationV1,
        StereoVerificationV1,
        CalibDataReplay,
        CalibDataStatistics
    };
    void setHomePage(Page home);

    void setDevMode(bool on);
    bool isDevMode() const;

    struct Configs
    {
        // SN regex pattern for Calibration Pages.
        // TODO: Remove later. Each page takes care of its own regex as needed.
        std::string snPattern{};

        // Default home page
        MainWindow::Page homePage = MainWindow::Page::StereoCalibrationV1;

        // Default app mode
        bool devMode = false;
    };
    void setFromConfigs(const Configs &configs);

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    Q_DECLARE_PIMPL(MainWindow)
};

} // namespace tl
