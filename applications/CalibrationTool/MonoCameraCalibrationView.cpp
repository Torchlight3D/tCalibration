#include "MonoCameraCalibrationView.h"
#include "ui_MonoCameraCalibrationView.h"

#include <filesystem>
#include <fstream>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include <QJsonDocument>
#include <QMessageBox>
#include <QSettings>
#include <QtConcurrent/QtConcurrent>

#include <tCalib/IO/CalibrationIO>
#include <tCore/ThreadPool>
#include <tDevice/ImageDataRecorder>
#include <tDevice/ImageFrame>
#include <tDevice/MotionDataRecorder>

#include "gui/qtcvutils.h"
#include "gui/qtimageview.h"
#include "DataConsumer.h"
#include "DeviceListItem.h"
#include "DeviceWatchdog.h"
#include "MesClient.h"
#include "MonoCameraCalibrationTask.h"
#include "MonoCameraClient.h"
#include "ProductInfo.h"
#include "SharedUdpSocket.h"
#include "TaskInfo.h"

namespace tl {

namespace fs = std::filesystem;
using namespace Qt::Literals::StringLiterals;

Q_LOGGING_CATEGORY(MonoCalibration, "mono.calibration")

namespace {

inline auto viewWindowTitle()
{
    return MonoCameraCalibrationView::tr("Mono Camera Calibration");
}
} // namespace

namespace key {
constexpr char kHostAddresses[]{"host_addresses"};
}

///------- MonoDataConsumer starts from here
class MonoDataConsumer final : public DataConsumer
{
    Q_OBJECT

public:
    MonoDataConsumer(std::shared_ptr<MonoCameraCalibrationTask> task,
                     std::shared_ptr<Channel<MonoImage>> visualChannel,
                     std::shared_ptr<Channel<ImuData>> motionChannel,
                     const std::string& root);

    void setDeviceInfo(const DeviceInfo& info);
    void resetTask();

    void enableRecording(bool on);
    void enableProcessing(bool on);

signals:
    void requestShowImage(const QImage& image);

private:
    void consume(size_t channel) override;

private:
    std::shared_ptr<MonoCameraCalibrationTask> _task;
    std::shared_ptr<Channel<MonoImage>> _visualChannel;
    std::shared_ptr<Channel<ImuData>> _motionChannel;
    std::unique_ptr<ImageDataRecorder> _imageRecorder;
    std::unique_ptr<MotionDataRecorder> _motionRecorder;
    std::string _root;
    std::atomic<bool> _process{false};
    std::atomic<bool> _record{false};
};

MonoDataConsumer::MonoDataConsumer(
    std::shared_ptr<MonoCameraCalibrationTask> task,
    std::shared_ptr<Channel<MonoImage>> visualChannel,
    std::shared_ptr<Channel<ImuData>> motionChannel, const std::string& root)
    : DataConsumer(2),
      _task(task),
      _visualChannel(visualChannel),
      _motionChannel(motionChannel),
      _root(root)
{
}

void MonoDataConsumer::setDeviceInfo(const DeviceInfo& info)
{
    _task->setDeviceInfo(info);
}

void MonoDataConsumer::resetTask()
{
    _task->resetTaskInfo();
    _task->clearData();
}

void MonoDataConsumer::enableRecording(bool on)
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

void MonoDataConsumer::enableProcessing(bool on)
{
    if (_process == on) {
        return;
    }

    _process = on;
}

void MonoDataConsumer::consume(size_t index)
{
    switch (index) {
        case 0: {
            while (_running[0] && !_visualChannel->closed()) {
                MonoImage mono;
                *_visualChannel >> mono;

                if (!mono.isValid()) {
                    continue;
                }

                QMetaObject::invokeMethod(
                    this,
                    [this, mono] {
                        emit requestShowImage(cvMatToQImage(mono.image));
                    },
                    Qt::QueuedConnection);

                if (_process) {
                    _task->addMonoData(mono);
                }

                if (_record) {
                    _imageRecorder->addFrame(
                        ImageFrame{.info = {.fx = 300.,
                                            .fy = 300.,
                                            .px = 320.,
                                            .py = 240.,
                                            .stamp = mono.timestamp,
                                            .cameraId = 0},
                                   .data = &(mono.image)});
                }
            }
        } break;
        case 1: {
            while (_running[1] && !_visualChannel->closed()) {
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

///------- MonoCameraCalibrationViewPrivate starts from here
class MonoCameraCalibrationViewPrivate
{
    Q_DECLARE_TR_FUNCTIONS(tl::MonoCameraCalibrationView)
    Q_DEFINE_PIMPL(MonoCameraCalibrationView)

public:
    explicit MonoCameraCalibrationViewPrivate(MonoCameraCalibrationView* q);
    ~MonoCameraCalibrationViewPrivate();

    void init();

    /// Slots
    void toggleService(bool on);
    void toggleStreaming(bool on);
    void toggleCapturing(bool on);

    /// Task
    void handleStreamStopped();
    void startCalibration();
    QCoro::Task<void> handleResults();

    /// GUI
    void updateUiByDevMode(bool on);
    void updateConnectionStatus(const QList<bool>& status);

    void appendDevice(bool updateUi = true);
    void removeDevice(int pos);

    bool uploadResults() const;
    bool useMes() const;

public:
    Ui::MonoCameraCalibrationView* ui;
    QtListWidget* m_deviceList;
    std::unique_ptr<MesClient> m_mesClient;
    std::shared_ptr<SharedUdpSocket> m_localHost;
    DeviceWatcher m_deviceWatcher;
    QPointer<DeviceListItem> m_lastDeviceItem; // Not owned

    // These lists should have same size, and same order
    QList<DeviceListItem*> m_deviceItems; // Not owned
    QWidgetList m_liveViews;              // Owned
    std::vector<std::shared_ptr<Channel<MonoImage>>> m_visualChannels;
    std::vector<std::shared_ptr<Channel<ImuData>>> m_motionChannels;
    std::vector<std::unique_ptr<MonoDataConsumer>> m_consumers;
    std::vector<std::unique_ptr<MonoCameraClient>> m_clients;
    std::vector<std::shared_ptr<MonoCameraCalibrationTask>> m_tasks;

    // Options
    MonoCameraCalibrationTask::Options m_taskOpts;
    MonoCameraCalibrationTask::Reference m_taskRef;
    ProductInfo m_product;
    std::string m_snPattern{};
    std::string m_targetConfigs;
    bool m_devMode{false};

    // The user could stream only to see the live view, then under this
    // occasion, calibration should not be triggered.
    bool m_captured{false};
};

MonoCameraCalibrationViewPrivate::MonoCameraCalibrationViewPrivate(
    MonoCameraCalibrationView* q)
    : q_ptr(q),
      ui(new Ui::MonoCameraCalibrationView),
      m_deviceList(new QtListWidget(q)),
      m_localHost(std::make_shared<SharedUdpSocket>())
{
}

MonoCameraCalibrationViewPrivate::~MonoCameraCalibrationViewPrivate()
{
    delete ui;
}

void MonoCameraCalibrationViewPrivate::init()
{
    Q_Q(MonoCameraCalibrationView);
    ui->setupUi(q);
    q->setWindowTitle(viewTitle(viewWindowTitle()));

    m_localHost->bindTo(kDefaultLocalAddr, kDefaultLocalMsgBusPort);

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

void MonoCameraCalibrationViewPrivate::toggleService(bool on)
{
    if (on) {
        // Start device watchdog
        QStringList hostAddresses;
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto& item = m_deviceItems[i];
            const auto& client = m_clients[i];

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
        m_deviceWatcher.blockSignals(true);
        emit m_deviceWatcher.stopWatch();

        // TODO: Stop data streaming
        qDebug(MonoCalibration) << tr("Closing streaming data channel...");

        // TODO: Stop running task
        qDebug(MonoCalibration) << tr("Stopping running tasks...");

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
        ui->addDeviceBtn->setEnabled(false);
        ui->removeDeviceBtn->setEnabled(false);
        for (const auto& item : m_deviceItems) {
            item->resetState();
        }
    }
}

void MonoCameraCalibrationViewPrivate::toggleStreaming(bool on)
{
    if (on) {
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto& client = m_clients[i];
            const auto& item = m_deviceItems[i];
            const auto& consumer = m_consumers[i];

            consumer->resetTask();
            m_visualChannels[i]->resume();
            m_motionChannels[i]->resume();
            consumer->startAll();
            client->startStreamingMonoData();
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
            const auto& client = m_clients[i];

            m_visualChannels[i]->close();
            m_motionChannels[i]->close();
            m_consumers[i]->stopAll();
            client->stopStreamingMonoData();
            client->stopStreamingMotionData();

            m_deviceItems[i]->setLedState(QtLedIndicator::State::Idle);
        }

        {
            QSignalBlocker blocker{ui->captureBtn};
            ui->captureBtn->setChecked(false);
            ui->captureBtn->setEnabled(false);
        }

        // TODO: Stop running task
        qDebug(MonoCalibration) << tr("Stopping running tasks...");

        // Update GUI
        ui->streamBtn->setText(tr("Start Streaming"));
    }
}

void MonoCameraCalibrationViewPrivate::toggleCapturing(bool on)
{
    Q_Q(MonoCameraCalibrationView);
    if (on) {
        // Check if calibration should proceed
        // 1. Check SN. We need SN to work with MES, but even MES is disable, SN
        // is still required.
        // 2. Query MES should proceed.
        std::vector<QString> devices;
        devices.reserve(m_deviceItems.size());
        for (const auto& item : m_deviceItems) {
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
            qInfo(MonoCalibration) << tr("Fetching device status from MES...");

            QStringList errMsgs;
            for (size_t i{0}; i < devices.size(); ++i) {
                const auto& sn = devices[i];
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
        }

        // Real actions here
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto& item = m_deviceItems[i];
            const auto& consumer = m_consumers[i];

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
        // Stop everthing, automaticaly trigger calibration.
        for (qsizetype i{0}; i < m_deviceItems.size(); ++i) {
            // Alias
            const auto& client = m_clients[i];
            const auto& consumer = m_consumers[i];

            m_visualChannels[i]->close();
            m_motionChannels[i]->close();
            consumer->stopAll();
            client->stopStreamingMonoData();
            client->stopStreamingMotionData();
            consumer->enableRecording(false);
            consumer->enableProcessing(false);

            m_deviceItems[i]->setLedState(QtLedIndicator::State::Warning);
        }
    }
}

void MonoCameraCalibrationViewPrivate::handleStreamStopped()
{
    if (m_captured) {
        if (std::ranges::any_of(m_consumers, [](const auto& consumer) {
                return !consumer->stopped();
            })) {
            return;
        }

        qInfo(MonoCalibration)
            << tr("All data streaming stopped, start calibration.");
        startCalibration();
    }
    else {
        for (const auto& item : m_deviceItems) {
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

void MonoCameraCalibrationViewPrivate::startCalibration()
{
    Q_Q(MonoCameraCalibrationView);
    // 1. QtConcurrent
    auto future = QtConcurrent::mapped(
        m_tasks, [this](const auto& task) { return task->startCalibration(); });

    auto watcher = new QFutureWatcher<MonoCameraCalibrationTask::Error>(q);
    QObject::connect(
        watcher, &QFutureWatcher<MonoCameraCalibrationTask::Error>::finished, q,
        [this, q, watcher] {
            const auto results = watcher->future().results();

            const auto numDevices = m_deviceItems.size();
            for (qsizetype i{0}; i < numDevices; ++i) {
                qInfo(MonoCalibration)
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

QCoro::Task<void> MonoCameraCalibrationViewPrivate::handleResults()
{
    Q_Q(MonoCameraCalibrationView);

    const auto numTasks = m_tasks.size();

    std::vector<ReportSummary> summaries;
    summaries.reserve(numTasks);
    QStringList mesErrMsgs;
    mesErrMsgs.reserve(numTasks);
    for (size_t i{0}; i < numTasks; ++i) {
        const auto& task = m_tasks[i];
        const auto& client = m_clients[i];
        const auto displayIndex = i + 1;

        if (!task->calibrated()) {
            qInfo(MonoCalibration)
                << tr("Skip Device %1: task is not calibrated.")
                       .arg(displayIndex);
            summaries.push_back({});
            continue;
        }

        const auto summary = task->resultAsSummary(m_taskRef);
        summaries.push_back(summary);

        const auto result = task->resultAsYaml();

        const auto& uuid = task->info().uuid;

        const auto taskDir = io::taskDir({}, task->info().uuid);
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

        const auto& device = task->deviceInfo();
        if (device.isVirtual || !uploadResults()) {
            continue;
        }

        qInfo(MonoCalibration)
            << tr("Sending calibration result to Device %1...")
                   .arg(displayIndex);

        const auto sent = co_await client->sendCalibrationResult(result);
        if (!sent) {
            qWarning(MonoCalibration)
                << tr("Failed to send calibration result to Device "
                      "%1.")
                       .arg(displayIndex);
        }

        const auto generated = co_await client->genGdc();
        if (!generated) {
            qWarning(MonoCalibration)
                << tr("Failed to generate rectify map on Device %1.")
                       .arg(displayIndex);
        }

        if (!useMes()) {
            continue;
        }

        qInfo(MonoCalibration)
            << tr("Updating Device %1 status to MES...").arg(displayIndex);

        const auto sn = QString::fromStdString(device.sn);

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
            qWarning(MonoCalibration)
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

void MonoCameraCalibrationViewPrivate::updateUiByDevMode(bool on)
{
    for (auto& w : QWidgetList{ui->uploadResults, ui->useMes, ui->settingsBtn,
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

void MonoCameraCalibrationViewPrivate::updateConnectionStatus(
    const QList<bool>& status)
{
    if (status.size() != m_deviceItems.size()) {
        return;
    }

    for (auto i{0}; i < m_deviceItems.size(); ++i) {
        const auto& connected = status[i];
        const auto& item = m_deviceItems[i];
        const auto hostAddr = item->hostAddress();

        if (item->connected() == connected) {
            continue;
        }

        if (connected) {
            qInfo(MonoCalibration)
                << tr("Device %1 is connected.").arg(hostAddr);
        }
        else {
            qWarning(MonoCalibration)
                << tr("Device %1 is disconnected.").arg(hostAddr);
        }

        item->setConnected(connected);
    }
}

void MonoCameraCalibrationViewPrivate::appendDevice(bool updateUi)
{
    Q_Q(MonoCameraCalibrationView);
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

    auto monoChannel = std::make_shared<Channel<MonoImage>>();
    m_visualChannels.push_back(monoChannel);

    auto motionChannel = std::make_shared<Channel<ImuData>>();
    m_motionChannels.push_back(motionChannel);

    auto task = std::make_shared<MonoCameraCalibrationTask>(m_taskOpts);
    if (!m_targetConfigs.empty()) {
        task->setTargetFromJson(m_targetConfigs);
    }
    task->setProductInfo(m_product);
    m_tasks.push_back(task);

    auto consumer = std::make_unique<MonoDataConsumer>(
        task, monoChannel, motionChannel, fs::current_path().string());
    q->connect(consumer.get(), &MonoDataConsumer::requestShowImage, liveView,
               &QtImageView::showImage);
    q->connect(consumer.get(), &MonoDataConsumer::allStopped, q,
               [this]() { handleStreamStopped(); });
    m_consumers.push_back(std::move(consumer));

    auto client = std::make_unique<MonoCameraClient>();
    client->bindMonoDataChannel(monoChannel);
    client->bindMotionDataChannel(motionChannel);
    client->bindSharedSocket(m_localHost);
    m_clients.push_back(std::move(client));
}

void MonoCameraCalibrationViewPrivate::removeDevice(int pos)
{
    // ...
}

bool MonoCameraCalibrationViewPrivate::uploadResults() const
{
    return ui->uploadResults->isChecked();
}

bool MonoCameraCalibrationViewPrivate::useMes() const
{
    return ui->useMes->isChecked() && m_mesClient && m_mesClient->isValid();
}

MonoCameraCalibrationView::MonoCameraCalibrationView(QWidget* parent)
    : ToolView(parent), d_ptr(new MonoCameraCalibrationViewPrivate(this))
{
    Q_D(MonoCameraCalibrationView);
    d->init();

    connect(
        &d->m_deviceWatcher, &DeviceWatcher::statusUpdated, this,
        [d](const QList<bool>& status) { d->updateConnectionStatus(status); });
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

MonoCameraCalibrationView::~MonoCameraCalibrationView() = default;

void MonoCameraCalibrationView::setDevMode(bool on)
{
    Q_D(MonoCameraCalibrationView);
    if (d->m_devMode == on) {
        return;
    }

    d->m_devMode = on;
    d->updateUiByDevMode(on);
}

bool MonoCameraCalibrationView::setFromJson(const std::string& json)
{
    Q_D(MonoCameraCalibrationView);
    try {
        const auto j = nlohmann::json::parse(json);

        j["sn_pattern"].get_to(d->m_snPattern);
        for (auto& item : d->m_deviceItems) {
            if (item) {
                item->setSerialNumberPattern(
                    QString::fromStdString(d->m_snPattern));
            }
        }

        std::string productName;
        j["product"].get_to(productName);
        if (const auto info = ProductInfo::of(productName)) {
            d->m_product = info.value();
            for (auto& task : d->m_tasks) {
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
        for (auto& task : d->m_tasks) {
            task->setTargetFromJson(d->m_targetConfigs);
        }

        j["task_options"].get_to(d->m_taskOpts);
        j["task_reference"].get_to(d->m_taskRef);
    }
    catch (const nlohmann::detail::parse_error& e) {
        qWarning(MonoCalibration)
            << tr("Failed to parse mono camera calibration view settings: ")
            << e.what();
        return false;
    }
    catch (const nlohmann::detail::out_of_range& e) {
        qWarning(MonoCalibration)
            << tr("Incompatible mono camera calibration view settings: ")
            << e.what();
        return false;
    }

    return true;
}

void MonoCameraCalibrationView::restoreSettings(QSettings& settings)
{
    Q_D(MonoCameraCalibrationView);

    settings.beginGroup("MonoCalibration");

    /// Host addresses
    if (const auto hostAddresses =
            settings.value(key::kHostAddresses, QStringList{}).toStringList();
        hostAddresses.empty()) {
        qInfo(MonoCalibration) << tr("No host to restore.");
    }
    else {
        qInfo(MonoCalibration)
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

void MonoCameraCalibrationView::saveSettings(QSettings& settings) const
{
    Q_D(const MonoCameraCalibrationView);

    settings.beginGroup("MonoCalibration");
    /// Host addresses
    QStringList hostAddresses;
    for (const auto& item : d->m_deviceItems) {
        hostAddresses << item->hostAddress();
    }
    settings.setValue("host_addresses", hostAddresses);

    settings.endGroup();
}

} // namespace tl

#include "MonoCameraCalibrationView.moc"
#include "moc_MonoCameraCalibrationView.cpp"
