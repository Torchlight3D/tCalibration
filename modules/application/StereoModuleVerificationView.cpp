#include "StereoModuleVerificationView.h"
#include "ui_StereoModuleVerificationView.h"

#include <filesystem>
#include <format>
#include <fstream>

#include <opencv2/imgcodecs.hpp>

#include <QDateTime>
#include <QtConcurrent/QtConcurrent>

#include <tCalibration/CalibrationIO>

#include "gui/QtOpenCV.h"
#include "gui/qimageutils.h"
#include "StereoModuleHubClient.h"
#include "StereoModuleVerificationConfigWidget.h"

namespace tl {

namespace fs = std::filesystem;

///------- StereoModuleVerificationViewPrivate starts from here
class StereoModuleVerificationViewPrivate
{
    AX_Q_DEFINE_PIMPL(StereoModuleVerificationView)

public:
    explicit StereoModuleVerificationViewPrivate(
        StereoModuleVerificationView *q);
    ~StereoModuleVerificationViewPrivate();

    void init();
    void updateUiByDevMode(bool on);

    /// Utils
    void showImage(const QImage &img);
    void showStereoImages(const QImage &left, const QImage &right);
    void showMessage(const QString &msg);
    void setIndicator(LedIndicator::Status status, const QColor &color);

public:
    Ui::StereoModuleVerificationView *ui;
    std::unique_ptr<StereoModuleHubClient> m_camClient;
    StereoCameraVerification::Options m_opts;
    std::string m_lastUuid{};
    int m_savedCount{0};
    bool m_devMode{false};
};

StereoModuleVerificationViewPrivate::StereoModuleVerificationViewPrivate(
    StereoModuleVerificationView *q)
    : q_ptr(q),
      ui(new Ui::StereoModuleVerificationView),
      m_camClient(StereoModuleHubClient::createDefault())
{
}

StereoModuleVerificationViewPrivate::~StereoModuleVerificationViewPrivate()
{
    delete ui;
}

void StereoModuleVerificationViewPrivate::init()
{
    Q_Q(StereoModuleVerificationView);
    ui->setupUi(q);
    q->setWindowTitle(
        StereoModuleVerificationView::tr("Stereo Module Verification"));

    ui->indicator->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    ui->indicator->setFixedSize(40, 30);

    // BOLD! everything
    auto font = q->font();
    font.setPointSize(15);
    font.setBold(true);
    for (auto w : QWidgetList{ui->verifyBtn}) {
        w->setFont(font);
    }

    font.setPointSize(13);
    font.setBold(false);
    for (auto w : QWidgetList{ui->status}) {
        w->setFont(font);
    }

    // Default options
    m_opts.chessPatternSize = {11, 8};
    m_opts.chessSize = {0.045, 0.045};
    auto &ref = m_opts.reference;
    ref.maxTrackerRejectRate = 0.;
    ref.maxEstimatorRejectRate = 0.;
    ref.maxStereoOptimizationCost = 20.;
    ref.maxRejectRelativeSizeMedian = 6e-3;
    ref.maxRejectRelativePositionMedian = 6e-3;
    ui->verifyResultView->setReference(ref);

    updateUiByDevMode(m_devMode);
}

void StereoModuleVerificationViewPrivate::updateUiByDevMode(bool on)
{
    ui->configBtn->setVisible(on);
}

void StereoModuleVerificationViewPrivate::showImage(const QImage &img)
{
    ui->imgView->showImage(img);
}

void StereoModuleVerificationViewPrivate::showStereoImages(const QImage &left,
                                                           const QImage &right)
{
    const auto merge = qimg::hconcat(left, right);
    if (merge.isNull()) {
        return;
    }

    ui->imgView->showImage(merge);
}

void StereoModuleVerificationViewPrivate::showMessage(const QString &msg)
{
    ui->status->setText(msg);
}

void StereoModuleVerificationViewPrivate::setIndicator(
    LedIndicator::Status status, const QColor &color)
{
    ui->indicator->setOnColor(color);
    ui->indicator->setStatus(status);
}

///------- StereoModuleVerificationView starts from here
StereoModuleVerificationView::StereoModuleVerificationView(QWidget *parent)
    : QWidget(parent), d_ptr(new StereoModuleVerificationViewPrivate(this))
{
    Q_D(StereoModuleVerificationView);
    d->init();

    /// Async call
    connect(
        this, &StereoModuleVerificationView::requestShowImage, this,
        [d](const QImage &img) { d->showImage(img); }, Qt::QueuedConnection);
    connect(
        this, &StereoModuleVerificationView::requestShowMessage, this,
        [d](const QString &msg) { d->showMessage(msg); }, Qt::QueuedConnection);
    connect(
        this, &StereoModuleVerificationView::requestUpdateIndicator, this,
        [d](bool pass) {
            d->setIndicator(LedIndicator::Status::On,
                            pass ? Qt::green : Qt::red);
        },
        Qt::QueuedConnection);
    connect(
        this, &StereoModuleVerificationView::requestShowStereoImages, this,
        [d](const QImage &left, const QImage &right) {
            d->showStereoImages(left, right);
        },
        Qt::QueuedConnection);
    connect(
        this, &StereoModuleVerificationView::requestShowSummary, this,
        [d](const StereoCameraVerification::Summary &summary) {
            d->ui->verifyResultView->showSummary(summary);
        },
        Qt::QueuedConnection);

    connect(d->ui->verifyBtn, &QAbstractButton::clicked, this,
            &StereoModuleVerificationView::startVerify);
    connect(d->ui->configBtn, &QAbstractButton::clicked, this, [this, d]() {
        StereoModuleVerificationConfigDialog configs{this};
        configs.widget()->setDefaultValues(d->m_opts);
        if (!configs.exec()) {
            return;
        }

        d->m_opts = configs.widget()->options();
        d->ui->verifyResultView->setReference(d->m_opts.reference);
    });
}

StereoModuleVerificationView::~StereoModuleVerificationView() = default;

bool StereoModuleVerificationView::setFromJson(const QJsonObject &jo)
{
    // TODO
    return false;
}

void StereoModuleVerificationView::setDevMode(bool on)
{
    Q_D(StereoModuleVerificationView);
    if (d->m_devMode == on) {
        return;
    }

    d->m_devMode = on;
    d->updateUiByDevMode(on);
}

void StereoModuleVerificationView::startVerify()
{
    Q_D(StereoModuleVerificationView);

    auto connectToServer = [this, d]() -> int {
        if (d->m_camClient->isChannelReady()) {
            emit requestShowMessage(
                tr("Server is ready, searching devices..."));
            return d->m_camClient->onlineDeviceCount();
        }

        emit requestShowMessage(tr("Server is not ready, try to reconnect..."));
        if (d->m_camClient->tryToConnect()) {
            emit requestShowMessage(
                tr("Server reconnected, searching devices..."));
            return d->m_camClient->onlineDeviceCount();
        }

        return -1;
    };

    auto retrieveParametersAndStereoData =
        [d, this](int deviceCount) -> std::tuple<std::string, StereoImageData> {
        // Use the first available device
        if (deviceCount < 1) {
            emit requestShowMessage(tr("Failed to reconnect server."));
            return {};
        }

        emit requestShowMessage(tr("%1 devices available, use the first one.")
                                    .arg(QString::number(deviceCount)));
        const auto &indices = d->m_camClient->onlineDeviceIndices();
        const auto &first = indices.front();

        emit requestShowMessage(tr("Pulling camera parameters from device."));
        std::string params;
        if (!d->m_camClient->pullCameraCalibrationFromDevice(params, first)) {
            emit requestShowMessage(
                tr("Failed to retrieve calibration parameters from device."));
            return {};
        }

        const auto stereo = d->m_camClient->getStereoImage(first);
        return std::make_tuple(params, stereo);
    };

    auto verifyImpl = [d, this](const std::tuple<std::string, StereoImageData>
                                    &paramsAndStereo) {
        const auto &[params, stereo] = paramsAndStereo;
        if (params.empty()) {
            emit requestShowMessage(tr("Invalid calibration parameters: "
                                       "Empty file."));
            emit requestUpdateIndicator(false);
            return;
        }
        if (!stereo.isValid()) {
            emit requestShowMessage(tr("Invalid stereo images: "
                                       "Empty images."));
            emit requestUpdateIndicator(false);
            return;
        }

        StereoCameraVerification::VerifyData verifyData;
        std::string uuid;
        if (!StereoCameraVerification::VerifyData::loadFromText(
                params, verifyData, uuid)) {
            emit requestShowMessage(tr("Failed to load calibration parameters: "
                                       "Invalid file format."));
            emit requestUpdateIndicator(false);
            return;
        }

        // Try to tell if the incoming parameter is from a new device
        if (uuid != d->m_lastUuid) {
            d->m_lastUuid = uuid;
            d->m_savedCount = 0;
        }

        StereoCameraVerification verification{verifyData, d->m_opts};
        const auto summary =
            verification.startVerify(stereo.left, stereo.right);

        using Error = StereoCameraVerification::Summary::Error;
        const auto &error = summary.error;
        if (error == Error::InvalidImage) {
            emit requestShowMessage(tr("Failed to verify results: "
                                       "Invalid input images."));
            emit requestUpdateIndicator(false);
            emit requestShowStereoImages(cvMatCopyToQImage(stereo.left),
                                         cvMatCopyToQImage(stereo.right));
            return;
        }
        if (error == Error::FailedFindChessboardCorner) {
            emit requestShowMessage(tr("Failed to verify results: "
                                       "Failed to find chessboard corners."));
            emit requestUpdateIndicator(false);
            emit requestShowStereoImages(cvMatCopyToQImage(stereo.left),
                                         cvMatCopyToQImage(stereo.right));
            return;
        }

        cv::Mat resultImg;
        verification.drawResult(resultImg);

        // Save a copy of parameters and current sampled data
        if (!d->m_lastUuid.empty()) {
            const auto taskDir =
                fs::path{io::kSaveDataDir} / io::taskDirName(d->m_lastUuid);
            fs::create_directories(taskDir);

            // Calibration result copy
            auto makeLocalCalibFilename = [](const std::string &orgin) {
                constexpr char kVerifyPrefix[]{"verification"};
                return std::format("{0}-{1}", kVerifyPrefix, orgin);
            };

            {
                std::ofstream fout{
                    taskDir /
                    makeLocalCalibFilename(io::kLubaCalibParameterFilename)};
                fout << params;
            }

            // In case one task is going to be verified multiple times
            const auto suffix = QDateTime::currentDateTime()
                                    .toString("yyMMddhhmmss")
                                    .toStdString();

            // Images
            auto imgName = [](int index, const std::string &hint,
                              const std::string &suffix) {
                return std::format("verify_{0}_{1}_{2}.png", hint, index,
                                   suffix);
            };

            cv::imwrite(
                (taskDir / imgName(d->m_savedCount, "left", suffix)).string(),
                stereo.left);
            cv::imwrite(
                (taskDir / imgName(d->m_savedCount, "right", suffix)).string(),
                stereo.right);

            // Report
            auto makeReportName = [](int index, const std::string &suffix) {
                return std::format("verification_report_{0}_{1}.txt", index,
                                   suffix);
            };

            {
                std::ofstream fout{taskDir /
                                   makeReportName(d->m_savedCount, suffix)};
                fout << StereoModuleVerifyResultView::verifySummaryToReport(
                            d->m_opts.reference, summary)
                            .toStdString();
            }

            ++d->m_savedCount;
        }

        emit requestShowSummary(summary);
        emit requestShowMessage(tr("%1 See the details below.")
                                    .arg(summary.passed()
                                             ? tr("Verification passed.")
                                             : tr("Verification failed.")));
        emit requestUpdateIndicator(summary.passed());
        emit requestShowImage(cvMatCopyToQImage(resultImg));
    };

    auto future = QtConcurrent::run(connectToServer)
                      .then(retrieveParametersAndStereoData)
                      .then(verifyImpl);
}

} // namespace tl

#include "moc_StereoModuleVerificationView.cpp"
