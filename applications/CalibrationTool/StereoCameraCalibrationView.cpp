#include "StereoCameraCalibrationView.h"
#include "ui_StereoCameraCalibrationView.h"

#include <filesystem>
#include <fstream>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include <QJsonDocument>
#include <QMessageBox>
#include <QSettings>
#include <QtConcurrent/QtConcurrent>

#include <tCalib/IO/CalibrationIO>
#include <tCamera/StereoCameraTypes>
#include <tCore/ThreadPool>
#include <tDevice/ImageDataRecorder>
#include <tDevice/ImageFrame>
#include <tDevice/MotionDataRecorder>

#include "gui/qtcvutils.h"
#include "gui/qtimageview.h"
#include "gui/qtwaitingspinner.h"
#include "DataConsumer.h"
#include "DeviceListItem.h"
#include "DeviceWatchdog.h"
#include "MesClient.h"
#include "ProductInfo.h"
#include "SharedUdpSocket.h"
#include "StereoCameraCalibrationTask.h"
#include "StereoCameraClient.h"
#include "TaskInfo.h"

namespace tl {

namespace fs = std::filesystem;
using namespace Qt::Literals::StringLiterals;

Q_LOGGING_CATEGORY(StereoCalibrationV2, "stereo.calibration.v2")

namespace {

inline auto viewWindowTitle()
{
    return StereoCameraCalibrationView::tr("Stereo Camera Calibration");
}
} // namespace

namespace key {
constexpr char kHostAddresses[]{"host_addresses"};
}

class StereoDataConsumer final : public DataConsumer
{
    Q_OBJECT

public:
    StereoDataConsumer(std::shared_ptr<StereoCameraCalibrationTask> task,
                       std::shared_ptr<Channel<StereoImages>> visualChannel,
                       std::shared_ptr<Channel<ImuData>> motionChannel,
                       const std::string &root);

    void setDeviceInfo(const DeviceInfo &info);
    void resetTask();

    void enableRecording(bool on);
    void enableProcessing(bool on);

signals:
    void requestShowImage(const QImage &image);

private:
    void consume(size_t channel) override;

private:
    std::shared_ptr<StereoCameraCalibrationTask> _task;
    std::shared_ptr<Channel<StereoImages>> _visualChannel;
    std::shared_ptr<Channel<ImuData>> _motionChannel;
    std::unique_ptr<ImageDataRecorder> _imageRecorder;
    std::unique_ptr<MotionDataRecorder> _motionRecorder;
    std::string _root;
    std::atomic<bool> _process{false};
    std::atomic<bool> _record{false};
};

StereoDataConsumer::StereoDataConsumer(
    std::shared_ptr<StereoCameraCalibrationTask> task,
    std::shared_ptr<Channel<StereoImages>> visualChannel,
    std::shared_ptr<Channel<ImuData>> motionChannel, const std::string &root)
    : DataConsumer(2),
      _task(task),
      _visualChannel(visualChannel),
      _motionChannel(motionChannel),
      _root(root)
{
}

void StereoDataConsumer::setDeviceInfo(const DeviceInfo &info)
{
    _task->setDeviceInfo(info);
}

void StereoDataConsumer::resetTask()
{
    _task->resetTaskInfo();
    _task->clearData();
}

void StereoDataConsumer::enableRecording(bool on)
{
    if (_record == on) {
        return;
    }

    if (on) {
        const auto sensorsDir = io::sensorsDir(_root, _task->info().uuid);
        _imageRecorder =
            std::make_unique<ImageDataRecorder>(sensorsDir.string());
        _motionRecorder = std::make_unique<MotionDataRecorder>(
            (sensorsDir / "imu.csv").string());

        _record = true;
    }
    else {
        _record = false;

        if (_imageRecorder) {
            _imageRecorder->close();
        }
        if (_motionRecorder) {
            _motionRecorder->close();
        }
    }
}

void StereoDataConsumer::enableProcessing(bool on)
{
    if (_process == on) {
        return;
    }

    _process = on;
}

void StereoDataConsumer::consume(size_t index)
{
    switch (index) {
        case 0: {
            while (_running[0] && !_visualChannel->closed()) {
                StereoImages stereo;
                *_visualChannel >> stereo;

                if (!stereo.isValid()) {
                    continue;
                }

                cv::Mat merged;
                cv::hconcat(stereo.left, stereo.right, merged);

                QMetaObject::invokeMethod(
                    this,
                    [this, merged] {
                        emit requestShowImage(cvMatToQImage(merged));
                    },
                    Qt::QueuedConnection);

                if (_process) {
                    _task->addStereoData(stereo);
                }

                if (_record) {
                    _imageRecorder->addFrame(
                        ImageFrame{.info = {.fx = 300.,
                                            .fy = 300.,
                                            .px = 320.,
                                            .py = 240.,
                                            .stamp = stereo.timestamp,
                                            .cameraId = kCameraLeftId},
                                   .data = &(stereo.left)});
                    _imageRecorder->addFrame(
                        ImageFrame{.info = {.fx = 1001.,
                                            .fy = 1001.,
                                            .px = 640.,
                                            .py = 360.,
                                            .stamp = stereo.timestamp,
                                            .cameraId = kCameraRightId},
                                   .data = &(stereo.right)});
                }
            }
        } break;
        case 1: {
            while (_running[1] && !_motionChannel->closed()) {
                ImuData imu;
                *_motionChannel >> imu;

                if (_record) {
                    _motionRecorder->append(imu);
                }

                if (_process) {
                    _task->addMotionData(imu);
                }
            }
        } break;
        default:
            break;
    }
}

///------- StereoCameraCalibrationViewPrivate starts from here
class StereoCameraCalibrationViewPrivate
{
    Q_DECLARE_TR_FUNCTIONS(tl::StereoCameraCalibrationView)
    Q_DEFINE_PIMPL(StereoCameraCalibrationView);

public:
    explicit StereoCameraCalibrationViewPrivate(StereoCameraCalibrationView *q);
    ~StereoCameraCalibrationViewPrivate();

    void init();

    /// Slots
    void toggleService(bool on);
    void toggleStreaming(bool on);
    void toggleCapturing(bool on);

    /// Task
    void handleStreamStopped();
    void startCalibration();
    QCoro::Task<> handleResults();

    /// GUI
    void updateUiByDevMode(bool on);
    void updateConnectionStatus(const QList<bool> &status);

    void appendDevice(bool updateUi = true);
    void removeDevice(int pos);

    bool uploadResults() const;
    bool useMes() const;

public:
    Ui::StereoCameraCalibrationView *ui;
    QtWaitingSpinner *m_waitingSpinner{nullptr};
    QtListWidget *m_deviceList;
    std::unique_ptr<MesClient> m_mesClient;
    std::shared_ptr<SharedUdpSocket> m_localHost;
    DeviceWatcher m_deviceWatcher;
    QPointer<DeviceListItem> m_lastDeviceItem; // Not owned

    // These lists should have same size, and same order
    QList<DeviceListItem *> m_deviceItems; // Not owned
    QWidgetList m_liveViews;               // Owned
    std::vector<std::shared_ptr<Channel<StereoImages>>> m_visualChannels;
    std::vector<std::shared_ptr<Channel<ImuData>>> m_motionChannels;
    std::vector<std::unique_ptr<StereoDataConsumer>> m_consumers;
    std::vector<std::unique_ptr<StereoCameraClient>> m_clients;
    std::vector<std::shared_ptr<StereoCameraCalibrationTask>> m_tasks;

    // Options
    StereoCameraCalibrationTask::Options m_taskOpts;
    StereoCameraCalibrationTask::Reference m_taskRef;
    ProductInfo m_product;
    std::string m_snPattern{};
    std::string m_targetConfigs;
    bool m_devMode{false};

    // The user could stream only to see the live view, then under this
    // occasion, calibration should not be triggered.
    bool m_captured{false};
};

StereoCameraCalibrationViewPrivate::StereoCameraCalibrationViewPrivate(
    StereoCameraCalibrationView *q)
    : q_ptr(q),
      ui(new Ui::StereoCameraCalibrationView),
      m_deviceList(new QtListWidget(q)),
      m_localHost(std::make_shared<SharedUdpSocket>())
{
}

StereoCameraCalibrationViewPrivate::~StereoCameraCalibrationViewPrivate()
{
    delete ui;
}

void StereoCameraCalibrationViewPrivate::init()
{
    Q_Q(StereoCameraCalibrationView);
    ui->setupUi(q);
    q->setWindowTitle(viewTitle(viewWindowTitle()));

    m_localHost->bindTo(kDefaultLocalAddr, kDefaultLocalMsgBusPort);
    m_waitingSpinner = new QtWaitingSpinner(ui->right);

    ui->left->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    ui->right->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    ui->splitter->setChildrenCollapsible(false);
    ui->splitter->setStretchFactor(ui->splitter->indexOf(ui->left), 8);
    ui->splitter->setStretchFactor(ui->splitter->indexOf(ui->right), 2);
    ui->tabLiveView->setLayoutStyle(QtGridWidget::FourViews);
    // ui->deviceBox->setSizePolicy(QSizePolicy::Preferred,
    // QSizePolicy::Minimum);
    // ui->resultBox->setSizePolicy(QSizePolicy::Preferred,
    //                              QSizePolicy::Expanding);
    ui->serviceBtn->setText(tr("Start Service"));
    ui->uploadResults->setChecked(true);
    ui->useMes->setChecked(true);
    ui->streamBtn->setEnabled(false);
    ui->captureBtn->setEnabled(false);
    m_deviceList->setPlaceholderText(tr("Add devices to connect..."));
    ui->deviceBox->layout()->addWidget(m_deviceList);

    updateUiByDevMode(m_devMode);
}

void StereoCameraCalibrationViewPrivate::toggleService(bool on)
{
    if (on) {
        // Start device watchdog
        QStringList hostAddresses;
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto &item = m_deviceItems[i];
            const auto &client = m_clients[i];

            // Not allowed to edit address after service started.
            item->lockHostAddress(true);

            const auto hostAddr = item->hostAddress();
            hostAddresses << hostAddr;
            client->connectTo(hostAddr.toStdString());
        }

        emit m_deviceWatcher.setAddresses(hostAddresses);
        m_deviceWatcher.blockSignals(false);
        emit m_deviceWatcher.startWatch();

        // Update GUI
        ui->serviceBtn->setText(tr("Stop Service"));
        ui->settingsBtn->setEnabled(false);
        ui->streamBtn->setEnabled(true);
        ui->captureBtn->setEnabled(false);
        ui->addDeviceBtn->setEnabled(false);
        ui->removeDeviceBtn->setEnabled(false);
    }
    else {
        // Disable button to prevent from more clicking
        ui->serviceBtn->setEnabled(false);

        // Turn off device watchdog
        // NOTE: After calling stopWatch(), the pending requests may still
        // trigger the update signal, so here we block its signal before calling
        // stopWatch()
        m_deviceWatcher.blockSignals(true);
        emit m_deviceWatcher.stopWatch();

        // TODO: Stop data streaming
        qDebug(StereoCalibrationV2) << tr("Closing streaming data channel...");

        // TODO: Stop running task
        qDebug(StereoCalibrationV2) << tr("Stopping running tasks...");

        // Update GUI
        ui->serviceBtn->setEnabled(true);
        ui->serviceBtn->setText(tr("Start Service"));
        ui->settingsBtn->setEnabled(true);
        {
            QSignalBlocker blocker{ui->streamBtn};
            ui->streamBtn->setChecked(false);
            ui->streamBtn->setEnabled(false);
        }
        {
            QSignalBlocker blocker{ui->captureBtn};
            ui->captureBtn->setChecked(false);
            ui->captureBtn->setEnabled(false);
        }
        ui->addDeviceBtn->setEnabled(true);
        ui->removeDeviceBtn->setEnabled(true);
        for (const auto &item : m_deviceItems) {
            item->resetState();
        }
    }
}

void StereoCameraCalibrationViewPrivate::toggleStreaming(bool on)
{
    if (on) {
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto &client = m_clients[i];
            const auto &item = m_deviceItems[i];
            const auto &consumer = m_consumers[i];

            consumer->resetTask();
            m_visualChannels[i]->resume();
            m_motionChannels[i]->resume();
            consumer->startAll();
            client->startStreamingStereoData();
            client->startStreamingMotionData();

            item->setLedState(QtLedIndicator::State::Success);
        }

        // Update GUI
        ui->streamBtn->setText(tr("Stop Streaming"));
        ui->captureBtn->setEnabled(true);
    }
    else {
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto &client = m_clients[i];

            m_visualChannels[i]->close();
            m_motionChannels[i]->close();
            m_consumers[i]->stopAll();
            client->stopStreamingStereoData();
            client->stopStreamingMotionData();

            m_deviceItems[i]->setLedState(QtLedIndicator::State::Idle);
        }

        {
            QSignalBlocker blocker{ui->captureBtn};
            ui->captureBtn->setChecked(false);
            ui->captureBtn->setEnabled(false);
        }

        // TODO: Stop running task
        qDebug(StereoCalibrationV2) << tr("Stopping running tasks...");

        // Update GUI
        ui->streamBtn->setText(tr("Start Streaming"));
    }
}

void StereoCameraCalibrationViewPrivate::toggleCapturing(bool on)
{
    Q_Q(StereoCameraCalibrationView);
    if (on) {
        // Check if calibration should proceed
        // 1. Check SN. We need SN to work with MES, but even MES is disable, SN
        // is still required.
        // 2. Query MES should proceed.
        std::vector<QString> devices;
        devices.reserve(m_deviceItems.size());
        for (const auto &item : m_deviceItems) {
            if (const auto sn = item->serialNumber(); sn.isEmpty()) {
                [[maybe_unused]] const auto _ = QMessageBox::warning(
                    q, tr("Calibration Error"),
                    tr("Not allowed to calibrate unknown devices."));

                {
                    QSignalBlocker blocker{ui->captureBtn};
                    ui->captureBtn->setChecked(false);
                }
                return;
            }
            else {
                devices.push_back(sn);
            }
        }

        if (useMes()) {
            qInfo(StereoCalibrationV2)
                << tr("Fetching device status from MES...");

            QStringList errMsgs;
            for (size_t i{0}; i < devices.size(); ++i) {
                const auto &sn = devices[i];
                if (const auto resp = m_mesClient->start(sn); !resp.result) {
                    errMsgs << tr("Device %1: %2")
                                   .arg(QString::number(i + 1), resp.msg);
                    LOG(WARNING) << "Device " << sn.toStdString()
                                 << " is not eligible to proceed calibration: "
                                 << resp.msg.toStdString();
                }
            }

            if (!errMsgs.empty()) {
                [[maybe_unused]] const auto _ = QMessageBox::warning(
                    q, tr("MES Error"),
                    u"%1\n%2"_s.arg(tr("Not all devices are eligible to "
                                       "proceed calibration."),
                                    errMsgs.join('\n')));

                {
                    QSignalBlocker blocker{ui->captureBtn};
                    ui->captureBtn->setChecked(false);
                }
                return;
            }

            qInfo(StereoCalibrationV2)
                << tr("All devices are eligible to proceed calibration.");
        }

        // Real actions here
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto &item = m_deviceItems[i];
            const auto &consumer = m_consumers[i];

            auto info = item->toInfo();
            info.name = m_product.name;
            consumer->setDeviceInfo(info);
            consumer->enableRecording(true);
            consumer->enableProcessing(true);
            item->setLedState(QtLedIndicator::State::Busy);
        }

        m_captured = true;

        // Update GUI
        ui->serviceBtn->setEnabled(false);
        ui->settingsBtn->setEnabled(false);
        ui->streamBtn->setEnabled(false);
        ui->captureBtn->setText(tr("Stop Capturing"));
        ui->addDeviceBtn->setEnabled(false);
        ui->removeDeviceBtn->setEnabled(false);
    }
    else {
        // Stop everything. After all the streams stop, calibration will be
        // triggered depeneds on user's stop intention.
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto &client = m_clients[i];
            const auto &consumer = m_consumers[i];

            m_visualChannels[i]->close();
            m_motionChannels[i]->close();
            consumer->stopAll();
            client->stopStreamingStereoData();
            client->stopStreamingMotionData();
            consumer->enableRecording(false);
            consumer->enableProcessing(false);

            m_deviceItems[i]->setLedState(QtLedIndicator::State::Warning);
        }
    }
}

void StereoCameraCalibrationViewPrivate::handleStreamStopped()
{
    if (m_captured) {
        if (std::ranges::any_of(m_consumers, [](const auto &consumer) {
                return !consumer->stopped();
            })) {
            return;
        }

        qInfo(StereoCalibrationV2)
            << tr("All data streaming stopped, start calibration.");
        startCalibration();
    }
    else {
        for (const auto &item : m_deviceItems) {
            item->lockHostAddress(false);
            item->lockSerialNumber(false);
            item->setLedState(QtLedIndicator::State::Idle);
        }
    }

    // Reset GUI
    ui->serviceBtn->setEnabled(true);
    ui->settingsBtn->setEnabled(true);
    {
        QSignalBlocker blocker{ui->streamBtn};
        ui->streamBtn->setEnabled(true);
        ui->streamBtn->setChecked(false);
        ui->streamBtn->setText(tr("Start Streaming"));
    }
    ui->captureBtn->setEnabled(false);
    ui->captureBtn->setText(tr("Start Capturing"));
}

void StereoCameraCalibrationViewPrivate::startCalibration()
{
    Q_Q(StereoCameraCalibrationView);
    // 1. QtConcurrent
    auto future = QtConcurrent::mapped(
        m_tasks, [this](const auto &task) { return task->startCalibration(); });

    auto watcher = new QFutureWatcher<StereoCameraCalibrationTask::Error>(q);
    QObject::connect(
        watcher, &QFutureWatcher<StereoCameraCalibrationTask::Error>::finished,
        q, [this, q, watcher] {
            const auto results = watcher->future().results();

            const auto numDevices = m_deviceItems.size();
            for (qsizetype i{0}; i < numDevices; ++i) {
                qInfo(StereoCalibrationV2)
                    << "Device " << i + 1 << " " << results[i];
            }

            handleResults();

            watcher->deleteLater();
        });
    watcher->setFuture(future);

    // 2. My thread pool
    // BS::thread_pool pool{static_cast<BS::concurrency_t>(m_tasks.size())};
    // pool.detach_loop<size_t>(size_t{0}, m_tasks.size(), [this](size_t i) {
    //     m_tasks[i]->startCalibration();
    // });
    // pool.wait();

    // handleResults();
}

QCoro::Task<> StereoCameraCalibrationViewPrivate::handleResults()
{
    // Logic:
    // + Calibrated or not?
    //   - Not calibrated
    //     1. Show empty summary (later), which is NG by default
    //   >>> Next
    //   - Calibrated
    //     + Show task summary (later)
    //     + Save result as yaml and report
    //     + Save debug infos (optional)
    // + Pass or NG?
    //   - NG
    //   >>> Next
    //   - Pass
    //     Real device or not?
    //     + Not real
    //     >>> Next
    //     + Real
    //       - Upload result to device
    //       - Upload result to MES
    // + Update GUI
    //   - Pass or NG
    //   - Summaries

    Q_Q(StereoCameraCalibrationView);

    const auto numTasks = m_tasks.size();

    std::vector<ReportSummary> summaries;
    summaries.reserve(numTasks);
    QStringList mesErrMsgs;
    mesErrMsgs.reserve(numTasks);
    for (size_t i{0}; i < numTasks; ++i) {
        const auto &task = m_tasks[i];
        const auto &client = m_clients[i];
        const auto displayIndex = i + 1;

        if (!task->calibrated()) {
            qInfo(StereoCalibrationV2)
                << tr("Skip Device %1: task is not calibrated.")
                       .arg(displayIndex);
            summaries.push_back({});
            continue;
        }

        const auto summary = task->resultAsSummary(m_taskRef);
        summaries.push_back(summary);

        const auto result = task->resultAsYaml();

        const auto &uuid = task->info().uuid;

        const auto taskDir = io::taskDir({}, uuid);
        fs::create_directories(taskDir);

        {
            std::ofstream fout{taskDir / io::kCalibCamParametersFilename};
            fout << result;
        }

        {
            std::ofstream fout{taskDir / io::kCalibrationReportName};
            fout << QJsonDocument{summary.toJson()}
                        .toJson(QJsonDocument::Indented)
                        .toStdString();
        }

        if (!summary.pass()) {
            continue;
        }

        const auto &device = task->deviceInfo();
        if (device.isVirtual || !uploadResults()) {
            continue;
        }

        qInfo(StereoCalibrationV2)
            << tr("Sending calibration result to Device %1...")
                   .arg(displayIndex);

        const auto sent = co_await client->sendCalibrationResult(result);
        if (!sent) {
            qWarning(StereoCalibrationV2)
                << tr("Failed to send calibration result to Device %1.")
                       .arg(displayIndex);
        }

        const auto generated = co_await client->genGdc();
        if (!generated) {
            qWarning(StereoCalibrationV2)
                << tr("Failed to generate rectify map on Device %1.")
                       .arg(displayIndex);
        }

        if (!useMes()) {
            continue;
        }

        qInfo(StereoCalibrationV2)
            << tr("Updating Device %1 status to MES...").arg(displayIndex);

        const auto sn = QString::fromStdString(device.sn);
        // NOTE: Line managers only care about pass or not, so we add
        // focal_length item here as placeholder.
        m_mesClient->appendItem(sn, {
                                        .name = "focal_length",
                                        .desc = {},
                                        .start = QDateTime::currentDateTime(),
                                        .end = QDateTime::currentDateTime(),
                                        .code = -1,
                                        .result = sent,
                                    });

        if (const auto resp = m_mesClient->finish(sn); !resp.result) {
            mesErrMsgs.push_back(resp.msg);
            qWarning(StereoCalibrationV2)
                << tr("Failed to upload Device %1 test results to MES. ")
                       .arg(displayIndex)
                << resp.msg;
            LOG(WARNING) << "Failed to upload Device " << device.sn
                         << " test results to MES: " << resp.msg.toStdString();
        }
    }

    ui->report->showSummaries(summaries);
    for (size_t i{0}; i < numTasks; ++i) {
        m_deviceItems[i]->setLedState(summaries[i].pass()
                                          ? QtLedIndicator::State::Success
                                          : QtLedIndicator::State::Failure);
    }

    if (!mesErrMsgs.empty()) {
        [[maybe_unused]] const auto _ = QMessageBox::warning(
            q, tr("MES Error"),
            u"%1\n%2"_s.arg(
                tr("Not all the test results are successfully uploaded."),
                mesErrMsgs.join('\n')));
    }
}

void StereoCameraCalibrationViewPrivate::updateUiByDevMode(bool on)
{
    for (auto &w : QWidgetList{ui->uploadResults, ui->useMes, ui->settingsBtn,
                               ui->addDeviceBtn, ui->removeDeviceBtn}) {
        w->setVisible(on);
        w->setEnabled(on);
    }

    if (on) {
        ui->uploadResults->setChecked(false);
        ui->useMes->setChecked(false);
    }
    else {
        ui->uploadResults->setChecked(true);
        ui->useMes->setChecked(true);
    }
}

void StereoCameraCalibrationViewPrivate::updateConnectionStatus(
    const QList<bool> &status)
{
    if (status.size() != m_deviceItems.size()) {
        return;
    }

    for (auto i{0}; i < m_deviceItems.size(); ++i) {
        const auto &connected = status[i];
        const auto &item = m_deviceItems[i];

        if (item->connected() == connected) {
            continue;
        }

        const auto hostAddr = item->hostAddress();
        if (connected) {
            qInfo(StereoCalibrationV2)
                << tr("Device %1 is connected.").arg(hostAddr);
        }
        else {
            qWarning(StereoCalibrationV2)
                << tr("Device %1 is disconnected.").arg(hostAddr);
        }

        item->setConnected(connected);
    }
}

void StereoCameraCalibrationViewPrivate::appendDevice(bool updateUi)
{
    Q_Q(StereoCameraCalibrationView);
    auto item = new DeviceListItem(m_deviceItems.size());
    if (!m_snPattern.empty()) {
        item->setSerialNumberPattern(QString::fromStdString(m_snPattern));
    }
    if (m_lastDeviceItem) {
        q->connect(m_lastDeviceItem, &DeviceListItem::serialNumberMatched, item,
                   &DeviceListItem::focusOnSerialNumber);
    }
    m_deviceItems << item;
    m_deviceList->addItem(item);
    m_lastDeviceItem = item;

    auto liveView = new QtImageView(q);
    m_liveViews << liveView;
    if (updateUi) {
        ui->tabLiveView->bindWidgets(m_liveViews);
    }

    auto stereoChannel = std::make_shared<Channel<StereoImages>>();
    m_visualChannels.push_back(stereoChannel);

    auto motionChannel = std::make_shared<Channel<ImuData>>();
    m_motionChannels.push_back(motionChannel);

    auto task = std::make_shared<StereoCameraCalibrationTask>(m_taskOpts);
    if (!m_targetConfigs.empty()) {
        task->setTargetFromJson(m_targetConfigs);
    }
    task->setProductInfo(m_product);
    m_tasks.push_back(task);

    auto consumer = std::make_unique<StereoDataConsumer>(
        task, stereoChannel, motionChannel, fs::current_path().string());
    q->connect(consumer.get(), &StereoDataConsumer::requestShowImage, liveView,
               &QtImageView::showImage);
    q->connect(consumer.get(), &StereoDataConsumer::allStopped, q,
               [this]() { handleStreamStopped(); });
    m_consumers.push_back(std::move(consumer));

    auto client = std::make_unique<StereoCameraClient>();
    client->bindStereoDataChannel(stereoChannel);
    client->bindMotionDataChannel(motionChannel);
    client->bindSharedSocket(m_localHost);
    m_clients.push_back(std::move(client));
}

void StereoCameraCalibrationViewPrivate::removeDevice(int pos)
{
    // ...
}

bool StereoCameraCalibrationViewPrivate::uploadResults() const
{
    return ui->uploadResults->isChecked();
}

bool StereoCameraCalibrationViewPrivate::useMes() const
{
    return ui->useMes->isChecked() && m_mesClient && m_mesClient->isValid();
}

///-------- StereoCameraCalibrationView starts from here
StereoCameraCalibrationView::StereoCameraCalibrationView(QWidget *parent)
    : ToolView(parent), d_ptr(new StereoCameraCalibrationViewPrivate(this))
{
    Q_D(StereoCameraCalibrationView);
    d->init();

    connect(
        &d->m_deviceWatcher, &DeviceWatcher::statusUpdated, this,
        [d](const QList<bool> &status) { d->updateConnectionStatus(status); });
    connect(d->ui->serviceBtn, &QAbstractButton::toggled, this,
            [d](bool on) { d->toggleService(on); });
    connect(d->ui->settingsBtn, &QAbstractButton::clicked, this, [this]() {
        // ...
    });
    connect(d->ui->streamBtn, &QAbstractButton::toggled, this,
            [d](bool on) { d->toggleStreaming(on); });
    connect(d->ui->captureBtn, &QAbstractButton::toggled, this,
            [d](bool on) { d->toggleCapturing(on); });
    connect(d->ui->addDeviceBtn, &QAbstractButton::clicked, this,
            [this, d]() { d->appendDevice(); });
    connect(d->ui->removeDeviceBtn, &QAbstractButton::clicked, this,
            [this, d]() {
                // TODO: Get selected item from list view
                d->removeDevice(0);
            });
}

StereoCameraCalibrationView::~StereoCameraCalibrationView() = default;

void StereoCameraCalibrationView::setDevMode(bool on)
{
    Q_D(StereoCameraCalibrationView);
    if (d->m_devMode == on) {
        return;
    }

    d->m_devMode = on;
    d->updateUiByDevMode(on);
}

bool StereoCameraCalibrationView::setFromJson(const std::string &json)
{
    Q_D(StereoCameraCalibrationView);
    try {
        const auto j = nlohmann::json::parse(json);

        j["sn_pattern"].get_to(d->m_snPattern);
        for (auto &item : d->m_deviceItems) {
            if (item) {
                item->setSerialNumberPattern(
                    QString::fromStdString(d->m_snPattern));
            }
        }

        std::string productName;
        j["product"].get_to(productName);
        if (const auto info = ProductInfo::of(productName)) {
            d->m_product = info.value();
            for (auto &task : d->m_tasks) {
                task->setProductInfo(d->m_product);
            }

            setWindowTitle(viewTitle(viewWindowTitle(),
                                     QString::fromStdString(productName)));
        }

        if (auto mes = MesClient::fromJson(
                QByteArray::fromStdString(j["mes"].dump()))) {
            d->m_mesClient = std::move(mes);
        }

        d->m_targetConfigs = j["target"].dump();
        for (auto &task : d->m_tasks) {
            task->setTargetFromJson(d->m_targetConfigs);
        }

        j["task_options"].get_to(d->m_taskOpts);
        j["task_reference"].get_to(d->m_taskRef);
    }
    catch (const nlohmann::detail::parse_error &e) {
        qWarning(StereoCalibrationV2)
            << tr("Failed to parse stereo camera calibration view settings: ")
            << e.what();
        return false;
    }
    catch (const nlohmann::detail::out_of_range &e) {
        qWarning(StereoCalibrationV2)
            << tr("Incompatible stereo camera calibration view settings: ")
            << e.what();
        return false;
    }

    return true;
}

void StereoCameraCalibrationView::restoreSettings(QSettings &settings)
{
    Q_D(StereoCameraCalibrationView);

    settings.beginGroup("StereoCalibrationV2");

    /// Host addresses
    if (const auto hostAddresses =
            settings.value(key::kHostAddresses, QStringList{}).toStringList();
        hostAddresses.empty()) {
        qInfo(StereoCalibrationV2) << tr("No host to restore.");
    }
    else {
        qInfo(StereoCalibrationV2)
            << tr("Restore host addresses: ") << hostAddresses;

        if (const auto more =
                hostAddresses.size() - d->m_deviceList->itemCount();
            more > 0) {
            for (auto i{0}; i < more; ++i) {
                d->appendDevice(false);
            }
            d->ui->tabLiveView->bindWidgets(d->m_liveViews);
        }

        for (size_t i{0}; i < hostAddresses.size(); ++i) {
            d->m_deviceItems[i]->setHostAddress(hostAddresses[i]);
        }
    }

    settings.endGroup();
}

void StereoCameraCalibrationView::saveSettings(QSettings &settings) const
{
    Q_D(const StereoCameraCalibrationView);

    settings.beginGroup("StereoCalibrationV2");
    /// Host addresses
    QStringList hostAddresses;
    for (const auto &item : d->m_deviceItems) {
        hostAddresses << item->hostAddress();
    }
    settings.setValue(key::kHostAddresses, hostAddresses);

    settings.endGroup();
}

} // namespace tl

#include "moc_StereoCameraCalibrationView.cpp"
#include "StereoCameraCalibrationView.moc"
