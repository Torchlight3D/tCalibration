#include "StereoCameraVerificationView.h"
#include "ui_StereoCameraVerificationView.h"

#include <filesystem>
#include <fstream>

#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include <QInputDialog>
#include <QMessageBox>
#include <QSettings>

#include <tCalib/StereoCameraVerification>
#include <tCalib/IO/CalibrationIO>
#include <tCore/TimeUtils>

#include "gui/qtcvutils.h"
#include "CalibTaskPlayer.h"
#include "DeviceWatchdog.h"
#include "MesClient.h"
#include "ProductInfo.h"
#include "StereoCameraClient.h"
#include "StereoModuleVerifyResultView.h"

namespace tl {

namespace fs = std::filesystem;

using namespace Qt::Literals::StringLiterals;
Q_LOGGING_CATEGORY(StereoVerificationV2, "stereo.verification.v2")

namespace {

inline auto viewWindowTitle()
{
    return StereoCameraVerificationView::tr("Stereo Camera Verification");
}
} // namespace

namespace key {
constexpr char kTag[]{"tag"};
constexpr char kHostAddress[]{"host_address"};
} // namespace key

class StereoCameraVerificationViewPrivate
{
    Q_DECLARE_TR_FUNCTIONS(tl::StereoCameraVerificationView)
    Q_DEFINE_PIMPL(StereoCameraVerificationView)

public:
    explicit StereoCameraVerificationViewPrivate(
        StereoCameraVerificationView* q);
    ~StereoCameraVerificationViewPrivate();

    void init();

    /// Slots
    void startWatchdog(bool on);
    QCoro::Task<void> handleVerify(const QString& sn);

    /// GUI
    void updateUiByDevMode(bool on);
    void updateConnectionStatus(const QList<bool>& status);

public:
    Ui::StereoCameraVerificationView* ui;

    DeviceWatcher m_deviceWatcher;
    bool m_connected{false};

    std::unique_ptr<StereoCameraClient> m_client;
    std::unique_ptr<MesClient> m_mesClient;

    StereoCameraVerification::Options m_opts;
    bool m_devMode{false};

    // Cache
    std::string m_lastUuid{};
};

StereoCameraVerificationViewPrivate::StereoCameraVerificationViewPrivate(
    StereoCameraVerificationView* q)
    : q_ptr(q),
      ui(new Ui::StereoCameraVerificationView),
      m_client(std::make_unique<StereoCameraClient>())
{
}

StereoCameraVerificationViewPrivate::~StereoCameraVerificationViewPrivate()
{
    delete ui;
}

void StereoCameraVerificationViewPrivate::init()
{
    Q_Q(StereoCameraVerificationView);
    ui->setupUi(q);
    q->setWindowTitle(viewTitle(viewWindowTitle()));

    m_client->bindTo(kDefaultLocalAddr, kDefaultLocalMsgBusPort);

    ui->imageView->setSizePolicy(QSizePolicy::Expanding,
                                 QSizePolicy::Preferred);
    ui->right->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    ui->splitter->setChildrenCollapsible(false);
    ui->splitter->setSizes({700, 300});
    ui->splitter->setStretchFactor(ui->splitter->indexOf(ui->imageView), 8);
    ui->splitter->setStretchFactor(ui->splitter->indexOf(ui->right), 2);
    ui->serviceBtn->setText(tr("Start Service"));
    ui->useMes->setChecked(true);
    ui->hostAddress->setConnected(false);
    // ui->verifyBtn->setEnabled(false);
    ui->status->setFixedHeight(15);
    ui->status->setFlat(true);
    ui->status->setOnColor(Qt::green);
    ui->status->setStatus(QtLedIndicator::Status::Off);

    updateUiByDevMode(m_devMode);
}

void StereoCameraVerificationViewPrivate::startWatchdog(bool on)
{
    if (on) {
        const auto hostAddr = ui->hostAddress->address();
        if (hostAddr.isEmpty()) {
            qWarning(StereoVerificationV2)
                << tr("No valid host address to watch.");
            {
                QSignalBlocker blocker{ui->serviceBtn};
                ui->serviceBtn->setChecked(false);
            }
            return;
        }

        emit m_deviceWatcher.setAddresses({hostAddr});
        emit m_deviceWatcher.startWatch();

        qInfo(StereoVerificationV2) << tr("Device monitoring initiated.");

        // Update GUI
        ui->serviceBtn->setText(tr("Stop Service"));
        ui->hostAddress->setEnabled(false);
    }
    else {
        emit m_deviceWatcher.stopWatch();

        // Updat GUI
        ui->serviceBtn->setText(tr("Start Service"));
        ui->hostAddress->setEnabled(true);
        ui->hostAddress->setConnected(false);
        ui->verifyBtn->setEnabled(false);
        ui->status->setState(QtLedIndicator::State::Idle);
    }
}

QCoro::Task<void> StereoCameraVerificationViewPrivate::handleVerify(
    const QString& sn)
{
    Q_Q(StereoCameraVerificationView);
    // const auto sn = ui->deviceSN->text();
    // if (sn.isEmpty()) {
    //     [[maybe_unused]] const auto _ =
    //         QMessageBox::warning(q, tr("Verification Error"),
    //                              tr("Not allowed to verify calibration result
    //                              "
    //                                 "of unknow device."));
    //     co_return;
    // }

    const auto useMes =
        ui->useMes->isChecked() && m_mesClient && m_mesClient->isValid();

    if (useMes) {
        if (const auto resp = m_mesClient->start(sn); !resp.result) {
            LOG(WARNING) << "Device " << sn.toStdString()
                         << " is not eligible to proceed verification: "
                         << resp.msg.toStdString();
            [[maybe_unused]] const auto _ = QMessageBox::warning(
                q, tr("MES Error"),
                u"%1\n%2"_s.arg(
                    tr("Device is not eligible to proceed verification."),
                    resp.msg));

            co_return;
        }
    }

    const auto hostAddr = ui->hostAddress->address();
    if (hostAddr.isEmpty()) {
        [[maybe_unused]] const auto _ =
            QMessageBox::warning(q, tr("Verification Error"),
                                 tr("Please enter the device address."));
        co_return;
    }

    m_client->connectTo(hostAddr.toStdString());

    const auto result = co_await m_client->retrieveCalibrationResult();

    if (result.empty()) {
        ui->status->setState(QtLedIndicator::State::Warning);
        qWarning(StereoVerificationV2)
            << tr("Failed to retrieve calibration result.");
        co_return;
    }

    StereoCameraVerification::VerifyData params;
    CalibMetaData info;
    if (!StereoCameraVerification::VerifyData::loadFromText(result, params,
                                                            info)) {
        ui->status->setState(QtLedIndicator::State::Warning);
        qWarning(StereoVerificationV2)
            << tr("Failed to parse retrieved calibration result.");
        co_return;
    }

    ui->refMinTemp->setValue(info.minTemp);
    ui->refMaxTemp->setValue(info.maxTemp);

    const auto stereo = m_client->snapStereoData();
    if (!stereo.isValid()) {
        ui->status->setState(QtLedIndicator::State::Warning);
        qWarning(StereoVerificationV2) << tr("Failed to retrieve stereo data.");
        co_return;
    }

    cv::Mat merged;
    cv::hconcat(stereo.left, stereo.right, merged);
    ui->imageView->showImage(cvMatCopyToQImage(merged));

    StereoCameraVerification verification{params, m_opts};
    const auto summary = verification.startVerify(stereo.left, stereo.right);
    const auto pass = summary.pass();

    using Error = StereoCameraVerification::Summary::Error;
    const auto& error = summary.errors;
    if (error == Error::InvalidImage) {
        ui->status->setState(QtLedIndicator::State::Failure);
        qWarning(StereoVerificationV2)
            << tr("Failed to verify calibration result: "
                  "Invalid input images.");
        co_return;
    }

    if (error == Error::FailedFindChessboardCorner) {
        ui->status->setState(QtLedIndicator::State::Failure);
        qWarning(StereoVerificationV2)
            << tr("Failed to verify calibration result: "
                  "Failed to find chessboard corners.");
        co_return;
    }

    ui->status->setState(pass ? QtLedIndicator::State::Success
                              : QtLedIndicator::State::Failure);

    const auto motion = m_client->snapMotionData();
    ui->currentTemp->setValue(motion.temperature);

    cv::Mat viz;
    verification.drawResult(viz);

    ui->imageView->showImage(cvMatCopyToQImage(viz));
    ui->report->showSummary(summary);

    // Try to tell if the incoming parameter is from a new device
    if (info.uuid != m_lastUuid) {
        m_lastUuid = info.uuid;
    }

    // Save a copy of parameters and current sampled data
    if (!m_lastUuid.empty()) {
        qInfo(StereoVerificationV2) << tr("Saving verification result...");

        const auto taskDir = io::taskDir({}, m_lastUuid);
        const auto verifyDir = taskDir / io::kVerifySamplesDir;
        fs::create_directories(verifyDir);

        // Calibration parameters
        {
            std::ofstream fout{taskDir / io::kVerifyCamParametersFilename};
            fout << result;
        }

        const auto currentDatetime = time::currentDatetime();
        const auto suffix = std::format("{:%y%m%d%H%M%OS}", currentDatetime);

        // Images
        auto imgName = [](const std::string& hint, const std::string& suffix) {
            return std::format("verify_{}_{}.png", hint, suffix);
        };

        cv::imwrite((verifyDir / imgName("left", suffix)).string(),
                    stereo.left);
        cv::imwrite((verifyDir / imgName("right", suffix)).string(),
                    stereo.right);

        // Report
        auto reportName = [](const std::string& suffix) {
            return std::format("{}_{}.tt", kVerifyReportPrefixV2, suffix);
        };

        {
            std::ofstream fout{verifyDir / reportName(suffix)};
            fout << StereoCameraVerification::toJson(
                m_opts.reference, summary,
                {.datetime = std::format("{:%F %X}", currentDatetime),
                 .tag = ui->tag->text().toStdString(),
                 .temperature = motion.temperature});
        }
    }

    if (useMes) {
        qInfo(StereoVerificationV2)
            << tr("Uploading verification result to MES...");

        m_mesClient->appendItem(
            sn, {.name = "Accuracy",
                 .desc = QString::number(summary.trackerRejectRate),
                 .start = QDateTime::currentDateTime(),
                 .end = QDateTime::currentDateTime(),
                 .code = -1,
                 .result = pass});

        if (const auto resp = m_mesClient->finish(sn); !resp.result) {
            LOG(WARNING) << "Failed to upload verification result of Device "
                         << sn.toStdString()
                         << " to MES: " << resp.msg.toStdString();
            qWarning(StereoVerificationV2)
                << tr("Failed to upload verification result to MES: ")
                << resp.msg;
            [[maybe_unused]] const auto _ = QMessageBox::warning(
                q, tr("MES Error"),
                u"%1\n%2"_s.arg(
                    tr("Failed to upload verification result to MES: "),
                    resp.msg));
        }
        else {
            qInfo(StereoVerificationV2)
                << tr("Verification result uploaded to MES.");
            [[maybe_unused]] const auto _ = QMessageBox::information(
                q, tr("MES Info"), tr("Verification result uploaded to MES."));
            ui->deviceSN->clear();
        }
    }

    qInfo(StereoVerificationV2) << tr("Verification finished.");
}

void StereoCameraVerificationViewPrivate::updateUiByDevMode(bool on)
{
    for (auto& w : QWidgetList{ui->useMes, ui->settingsBtn}) {
        w->setVisible(on);
        w->setEnabled(on);
    }

    if (on) {
        ui->useMes->setChecked(false);
        ui->tag->setEnabled(true);
    }
    else {
        ui->useMes->setChecked(true);
        ui->tag->setEnabled(false);
    }
}

void StereoCameraVerificationViewPrivate::updateConnectionStatus(
    const QList<bool>& status)
{
    if (status.size() != 1) {
        return;
    }

    const auto& connected = status[0];

    if (m_connected == connected) {
        return;
    }

    m_connected = connected;

    const auto hostAddr = ui->hostAddress->address();

    if (connected) {
        qInfo(StereoVerificationV2)
            << tr("Device %1 is connected.").arg(hostAddr);
        ui->verifyBtn->setEnabled(true);
        ui->hostAddress->setConnected(true);
    }
    else {
        qWarning(StereoVerificationV2)
            << tr("Device %1 is disconnected.").arg(hostAddr);
        ui->verifyBtn->setEnabled(false);
        ui->hostAddress->setConnected(false);
    }
}

StereoCameraVerificationView::StereoCameraVerificationView(QWidget* parent)
    : ToolView(parent), d_ptr(new StereoCameraVerificationViewPrivate(this))
{
    Q_D(StereoCameraVerificationView);
    d->init();

    connect(
        &d->m_deviceWatcher, &DeviceWatcher::statusUpdated, this,
        [d](const QList<bool>& status) { d->updateConnectionStatus(status); });

    connect(d->ui->serviceBtn, &QAbstractButton::toggled, this,
            [d](bool checked) { d->startWatchdog(checked); });
    connect(d->ui->verifyBtn, &QAbstractButton::clicked, this, [this, d]() {
        bool ok;
        const auto sn = QInputDialog::getText(
            this, tr("Please input device SN"), tr("Device SN:"),
            QLineEdit::Normal, {}, &ok, {}, Qt::ImhLatinOnly);

        if (!ok || sn.isEmpty()) {
            return;
        }

        d->ui->deviceSN->setText(sn);
        d->handleVerify(sn);
    });
}

StereoCameraVerificationView::~StereoCameraVerificationView() = default;

void StereoCameraVerificationView::setDevMode(bool on)
{
    Q_D(StereoCameraVerificationView);
    if (d->m_devMode == on) {
        return;
    }

    d->m_devMode = on;
    d->updateUiByDevMode(on);
}

bool StereoCameraVerificationView::setFromJson(const std::string& json)
{
    Q_D(StereoCameraVerificationView);
    try {
        const auto j = nlohmann::json::parse(json);

        std::string productName;
        j["product"].get_to(productName);
        if (const auto info = ProductInfo::of(productName)) {
            setWindowTitle(viewTitle(viewWindowTitle(),
                                     QString::fromStdString(productName)));
        }

        // Mes
        if (auto mes = MesClient::fromJson(
                QByteArray::fromStdString(j["mes"].dump()))) {
            d->m_mesClient = std::move(mes);
        }

        // TODO: Support more scenes (types)
        const auto j_configs = j["target"]["options"];
        d->m_opts.chessPatternSize = {j_configs["rows"].get<int>(),
                                      j_configs["cols"].get<int>()};
        d->m_opts.chessSize = {j_configs["row_spacing"].get<double>(),
                               j_configs["col_spacing"].get<double>()};

        j["verify_reference"].get_to(d->m_opts.reference);
    }
    catch (const nlohmann::detail::parse_error& e) {
        qWarning(StereoVerificationV2)
            << tr("Failed to parse stereo camera verification view settings: ")
            << e.what();
        return false;
    }
    catch (const nlohmann::detail::out_of_range& e) {
        qWarning(StereoVerificationV2)
            << tr("Incompatiable stereo camera verification view settings: ")
            << e.what();
        return false;
    }

    // TODO: Add method updateGuiByOptions
    d->ui->report->setReference(d->m_opts.reference);
    return true;
}

void StereoCameraVerificationView::restoreSettings(QSettings& settings)
{
    Q_D(StereoCameraVerificationView);

    settings.beginGroup("StereoVerificationV2");

    if (const auto hostAddr =
            settings.value(key::kHostAddress, QString{}).toString();
        hostAddr.isEmpty()) {
        qInfo(StereoVerificationV2) << tr("No host to restore.");
    }
    else {
        qInfo(StereoVerificationV2) << tr("Restore host address: ") << hostAddr;
        d->ui->hostAddress->setAddress(hostAddr);
    }

    d->ui->tag->setText(settings.value(key::kTag, QString{}).toString());

    settings.endGroup();
}

void StereoCameraVerificationView::saveSettings(QSettings& settings) const
{
    Q_D(const StereoCameraVerificationView);

    settings.beginGroup("StereoVerificationV2");

    const auto hostAddr = d->ui->hostAddress->address();
    settings.setValue(key::kHostAddress, hostAddr);

    settings.setValue(key::kTag, d->ui->tag->text());

    settings.endGroup();
}

} // namespace tl

#include "moc_StereoCameraVerificationView.cpp"
