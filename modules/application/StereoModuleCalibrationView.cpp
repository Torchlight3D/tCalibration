#include "StereoModuleCalibrationView.h"
#include "ui_StereoModuleCalibrationView.h"

#include <filesystem>
#include <format>

#include <opencv2/imgproc.hpp>

#include <QtConcurrent/QtConcurrent>
#include <QUuid>

#include <tCalibration/CalibrationIO>
#include <tCamera/StereoCameraTypes>
#include <tCore/Channel>
#include <tCore/ThreadPool>
#include <tDevice/StereoDataRecorder>

#include "gui/AutoDeleteThread.h"
#include "gui/QtOpenCV.h"
#include "gui/qimageutils.h"
#include "StereoModuleCalibrationConfigWidget.h"
#include "StereoModuleHubClient.h"
#include "StereoModuleTask.h"

namespace tl {

namespace fs = std::filesystem;

namespace {
constexpr char kPlayIconName[]{":/icons/play.svg"};
constexpr char kStopIconName[]{":/icons/stop.svg"};
constexpr double kMinReservedSpaceInGB{5.};

inline double gigabytesAvailable(const std::string &path)
{
    std::error_code fileError;
    const auto spaceInfo = fs::space(fs::path{path}, fileError);
    constexpr double b2Gb = 1. / 1024. / 1024. / 1024.;
    return fileError ? 0. : (spaceInfo.available * b2Gb);
}

} // namespace

///------- StereoModuleHubService starts from here
class StereoModuleHubService : public AutoDeleteThread<StereoModuleHubClient>
{
    Q_OBJECT

    using Base = AutoDeleteThread<StereoModuleHubClient>;

public:
    explicit StereoModuleHubService(QObject *parent = nullptr);

    /// Instant response interfaces
    int onlineDeviceCount();
    const std::vector<int> &onlineDeviceIndices();
    std::vector<StereoModuleInfo> onlineDeviceInfos();
    std::vector<StereoModuleInfo> deviceInfos();
    int devicePosOf(int index);
    int deviceIndexAt(int pos);

signals:
    /// Service
    void startBlockStreaming();
    void stopBlockStreaming();
    void startAsyncStreaming();
    void stopAsyncStreaming();
    void startCallbackStreaming();
    void stopCallbackStreaming();

    /// Status
    void stereoImageDataBufferUpdated(int index);
    void stereoImageDataReceived(const StereoImageData &data, int index);
    void imuDataReceived(const ImuData &data, int index);
    void imuDataListReceived(const std::vector<ImuData> &data, int index);
    void streamStereoFinished(int index, bool withError);
    void streamImuFinished(int index, bool withError);
};

StereoModuleHubService::StereoModuleHubService(QObject *parent)
    : Base(StereoModuleHubClient::createDefault(), parent)
{
    /// These are all Qt::QueueConnection
    connect(this, &StereoModuleHubService::startBlockStreaming, worker(),
            &StereoModuleHubClient::blockStartStreaming);
    connect(this, &StereoModuleHubService::stopBlockStreaming, worker(),
            &StereoModuleHubClient::blockStopStreaming);
    connect(this, &StereoModuleHubService::startAsyncStreaming, worker(),
            &StereoModuleHubClient::asyncStartStreaming);
    connect(this, &StereoModuleHubService::stopAsyncStreaming, worker(),
            &StereoModuleHubClient::asyncStopStreaming);
    connect(this, &StereoModuleHubService::startCallbackStreaming, worker(),
            &StereoModuleHubClient::callbackStartStreaming);
    connect(this, &StereoModuleHubService::stopCallbackStreaming, worker(),
            &StereoModuleHubClient::callbackStopStreaming);

    connect(worker(), &StereoModuleHubClient::stereoImageDataBufferUpdated,
            this, &StereoModuleHubService::stereoImageDataBufferUpdated);
    connect(worker(), &StereoModuleHubClient::stereoImageDataReceived, this,
            &StereoModuleHubService::stereoImageDataReceived);
    connect(worker(), &StereoModuleHubClient::imuDataReceived, this,
            &StereoModuleHubService::imuDataReceived);
    connect(worker(), &StereoModuleHubClient::imuDataListReceived, this,
            &StereoModuleHubService::imuDataListReceived);
    connect(worker(), &StereoModuleHubClient::streamStereoFinished, this,
            &StereoModuleHubService::streamStereoFinished);
    connect(worker(), &StereoModuleHubClient::streamImuFinished, this,
            &StereoModuleHubService::streamImuFinished);
}

int StereoModuleHubService::onlineDeviceCount()
{
    return worker()->onlineDeviceCount(false);
}

const std::vector<int> &StereoModuleHubService::onlineDeviceIndices()
{
    return worker()->onlineDeviceIndices(false);
}

std::vector<StereoModuleInfo> StereoModuleHubService::onlineDeviceInfos()
{
    return worker()->onlineDeviceInfos(false);
}

std::vector<StereoModuleInfo> StereoModuleHubService::deviceInfos()
{
    return worker()->deviceInfos(false);
}

int StereoModuleHubService::devicePosOf(int index)
{
    return worker()->devicePosOf(index, false);
}

int StereoModuleHubService::deviceIndexAt(int pos)
{
    return worker()->deviceIndexAt(pos, false);
}

///------- CalibTaskManager starts from here
namespace {
constexpr int kPreprocessThreadCount{2};
}
class CalibTaskManager : public QObject
{
    Q_OBJECT

public:
    using Tasks = std::vector<StereoModuleTask::Ptr>;
    using Recorders = std::vector<StereoDataRecorder::Ptr>;

    explicit CalibTaskManager(QObject *parent = nullptr);

    /// Data
    const Tasks &tasks() const;
    bool saveData() const;

signals:
    /// States
    void prepareTaskFinished();
    void preprocessingFinished();
    void allCalibrationFinished();

public slots:
    // Clear existed tasks if there's any
    void prepareTask(const std::vector<StereoModuleInfo> &infos);
    void processStereoImageData(const StereoImageData &data, int pos);
    void processMotionData(const ImuData &data, int pos);
    void processMotionDataList(const ImuDataList &data, int pos);
    void stopPreprocessing();
    void startCalibration();
    void stopCalibration();

private:
    void addStereoDataToTask(const StereoImageData &data, int pos);
    void addMotionDataToTask(const ImuData &data, int pos);
    void saveStereoData(const StereoImageData &data, int pos);
    void saveMotionData(const ImuData &data, int pos);

    bool isValidPos(int pos) const;

private:
    Tasks m_tasks;
    Recorders m_recorders;
    ThreadPool m_threadpool{kPreprocessThreadCount};
    bool m_saveData{true};
};

CalibTaskManager::CalibTaskManager(QObject *parent) : QObject(parent) {}

const CalibTaskManager::Tasks &CalibTaskManager::tasks() const
{
    return m_tasks;
}

bool CalibTaskManager::saveData() const { return m_saveData; }

void CalibTaskManager::prepareTask(const std::vector<StereoModuleInfo> &infos)
{
    auto createTask =
        [](const StereoModuleTask::Options &opts,
           const StereoModuleInfo &info) -> StereoModuleTask::Ptr {
        auto task = std::make_unique<StereoModuleTask>(opts);
        task->setUuid(
            QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString());
        task->setDeviceInfo(info);
        task->prepare();
        return task;
    };

    // Options can be generated from GUI
    StereoModuleTask::Options options;
    options.skipStep = 6;
    options.maxIntrinsicsViewCount = 50;
    options.minIntrinsicsViewCount = 10;
    options.maxViewCount = 700;
    options.minViewCount = 300;
    options.taskType = StereoModuleTask::CalibTaskType::CameraImuRotation;

    const fs::path saveDir{io::kSaveDataDir};
    const auto count = infos.size();

    m_tasks.clear();
    m_recorders.clear();

    for (size_t i{0}; i < count; ++i) {
        auto task = createTask(options, infos[i]);
        const auto &uuid = task->uuid();
        m_tasks.push_back(std::move(task));

        const auto taskDir = saveDir / io::taskDirName(uuid);
        m_recorders.push_back(
            std::move(StereoDataRecorder::create(taskDir.string())));
    }

    // FIXME: We can't use too many threads for preprocessing here, as
    // preprocessing will add detection into Task, which may lead to race
    // condition.
    // To workaround, we use one less than total task count as thread count.
    // Better solutions are welcomed.
    m_threadpool.reset(std::max(size_t(1), count - 1));

    m_saveData = gigabytesAvailable(".") > kMinReservedSpaceInGB;

    emit prepareTaskFinished();
}

void CalibTaskManager::processStereoImageData(const StereoImageData &data,
                                              int pos)
{
    if (!isValidPos(pos)) {
        return;
    }

    addStereoDataToTask(data, pos);
    if (m_saveData) {
        saveStereoData(data, pos);
    }
}

void CalibTaskManager::processMotionData(const ImuData &data, int pos)
{
    if (!isValidPos(pos)) {
        return;
    }

    addMotionDataToTask(data, pos);
    if (m_saveData) {
        saveMotionData(data, pos);
    }
}

void CalibTaskManager::processMotionDataList(const ImuDataList &data, int pos)
{
    if (!isValidPos(pos)) {
        return;
    }

    for (const auto &item : data) {
        addMotionDataToTask(item, pos);
        saveMotionData(item, pos);
    }
}

void CalibTaskManager::stopPreprocessing()
{
    m_threadpool.waitForDone();
    emit preprocessingFinished();
}

void CalibTaskManager::startCalibration()
{
    // Method: Serial processing
    // for (auto &task : m_tasks) {
    //     if (task->readyToCalculate()) {
    //         task->startCalculation();
    //     }
    // }

    // Method: Paralleled for-loop
    m_threadpool.reset(m_tasks.size());
    m_threadpool
        .parallelize_loop(m_tasks.size(),
                          [this](size_t a, size_t b) {
                              for (auto i{a}; i < b; ++i) {
                                  if (m_tasks[i]->readyToCalculate()) {
                                      m_tasks[i]->startCalculation();
                                  }
                              }
                          })
        .wait();

    emit allCalibrationFinished();
}

void CalibTaskManager::stopCalibration()
{
    // TODO: Rarely used, do nothing now
    // m_threadpool.pause();
}

void CalibTaskManager::addStereoDataToTask(const StereoImageData &data, int pos)
{
    // Method: Thread pool processing
    // QUEST: Dont use std::ref(cref) here, why?
    m_threadpool.push_task(
        [this, pos](const StereoImageData &data) {
            m_tasks[pos]->addStereoData(data);
        },
        data);

    // Method: Serial processsing
    // m_tasks[pos]->addStereoData(data);
}

void CalibTaskManager::addMotionDataToTask(const ImuData &data, int pos)
{
    m_tasks[pos]->addMotionData(data);
}

void CalibTaskManager::saveStereoData(const StereoImageData &data, int pos)
{
    const FrameData left{.t = data.timestamp,
                         .focalLengthX = 1000.,
                         .focalLengthY = 1000.,
                         .px = 640.,
                         .py = 360.,
                         .cameraId = kCameraLeftId,
                         .frameData = &(data.left)};
    const FrameData right{.t = data.timestamp,
                          .focalLengthX = 1001.,
                          .focalLengthY = 1001.0,
                          .px = 640.0,
                          .py = 360.0,
                          .cameraId = kCameraRightId,
                          .frameData = &(data.right)};

    m_recorders[pos]->addFrame(left);
    m_recorders[pos]->addFrame(right);
}

void CalibTaskManager::saveMotionData(const ImuData &data, int pos)
{
    m_recorders[pos]->addMotionData(data);
}

bool CalibTaskManager::isValidPos(int pos) const
{
    return pos < m_tasks.size() && pos >= 0;
}

///------- CalibTaskService starts from here
class CalibTaskService : public AutoDeleteThread<CalibTaskManager>
{
    Q_OBJECT

    using Base = AutoDeleteThread<CalibTaskManager>;

public:
    explicit CalibTaskService(QObject *parent = nullptr);

    const CalibTaskManager::Tasks &tasks() const { return worker()->tasks(); }
    bool saveData() const { return worker()->saveData(); }

signals:
    /// Services
    void prepareTask(const std::vector<StereoModuleInfo> &infos);
    void processStereoData(const QByteArray &left, const QByteArray &right,
                           double timestamp, int pos);
    void processStereoImageData(const StereoImageData &data, int pos);
    void processMotionData(const ImuData &data, int pos);
    void processMotionDataList(const ImuDataList &data, int pos);
    void streamingAboutToStop();
    void startCalibration();
    void stopCalibration();

    /// Status
    void prepareTaskFinished();
    void preprocessingFinished();
    void allCalibrationFinished();
};

CalibTaskService::CalibTaskService(QObject *parent)
    : Base(new CalibTaskManager(), parent)
{
    connect(this, &CalibTaskService::prepareTask, worker(),
            &CalibTaskManager::prepareTask);
    connect(this, &CalibTaskService::processStereoImageData, worker(),
            &CalibTaskManager::processStereoImageData);
    connect(this, &CalibTaskService::processMotionData, worker(),
            &CalibTaskManager::processMotionData);
    connect(this, &CalibTaskService::processMotionDataList, worker(),
            &CalibTaskManager::processMotionDataList);
    connect(this, &CalibTaskService::streamingAboutToStop, worker(),
            &CalibTaskManager::stopPreprocessing);
    connect(this, &CalibTaskService::startCalibration, worker(),
            &CalibTaskManager::startCalibration);
    connect(this, &CalibTaskService::stopCalibration, worker(),
            &CalibTaskManager::startCalibration);

    connect(worker(), &CalibTaskManager::prepareTaskFinished, this,
            &CalibTaskService::prepareTaskFinished);
    connect(worker(), &CalibTaskManager::preprocessingFinished, this,
            &CalibTaskService::preprocessingFinished);
    connect(worker(), &CalibTaskManager::allCalibrationFinished, this,
            &CalibTaskService::allCalibrationFinished);
}

///------- RemoteStatusModel starts from here
class RemoteStatusModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    enum RemoteStatus
    {
        AllFailed = 0x00,

        CamCalibrationSent = 0x01,
        UndistortRectifyMapSent = 0x02,
        UndistortRectifyMapBinGenerated = 0x04,
        CloudUploaded = 0x08,
    };

    enum class Section
    {
        CalibrationResult = 0,
        UndistortRectifyMap,
        UndistortRectifyMapBin,
        ToCloud,

        Count,
    };

    using QAbstractTableModel::QAbstractTableModel;

    int rowCount(const QModelIndex &parent) const override;
    int columnCount(const QModelIndex &parent) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    QVariant headerData(int section, Qt::Orientation orientation,
                        int role) const override;

    QVariant data(const QModelIndex &index, int role) const override;

    using Data = std::vector<RemoteStatus>;
    void show(const Data &data);

    inline void resetModel()
    {
        beginResetModel();
        endResetModel();
    }

private:
    static QString sectionName(Section section)
    {
        static const std::unordered_map<Section, QString> map{
            {Section::CalibrationResult, tr("Calibration Result")},
            {Section::UndistortRectifyMap, tr("Undistort Rectify Map")},
            {Section::UndistortRectifyMapBin, tr("Undistort Rectify Map Bin")},
            {Section::ToCloud, tr("To Cloud")}};
        return map.at(section);
    }

private:
    Data m_items;
};

MAKE_FLAGS(RemoteStatusModel::RemoteStatus)

int RemoteStatusModel::rowCount(const QModelIndex &parent) const
{
    return !parent.isValid() ? m_items.size() : 0;
}

int RemoteStatusModel::columnCount(const QModelIndex &parent) const
{
    return !parent.isValid() ? static_cast<int>(Section::Count) : 0;
}

Qt::ItemFlags RemoteStatusModel::flags(const QModelIndex &index) const
{
    if (!index.isValid()) {
        return Qt::ItemIsEnabled;
    }

    return QAbstractTableModel::flags(index);
}

QVariant RemoteStatusModel::headerData(int section, Qt::Orientation orientation,
                                       int role) const
{
    if (role != Qt::DisplayRole) {
        return {};
    }

    switch (orientation) {
        case Qt::Horizontal:
            return sectionName(static_cast<Section>(section));
        case Qt::Vertical:
            return tr("Device #%1").arg(QString::number(section + 1));
        default:
            break;
    }

    return {};
}

QVariant RemoteStatusModel::data(const QModelIndex &index, int role) const
{
    const int row = index.row();
    if (!index.isValid() || index.row() >= m_items.size()) {
        return {};
    }

    const auto &item = m_items.at(row);
    switch (role) {
        case Qt::DisplayRole: {
            auto statusToString = [](bool success) {
                return success ? tr("Success") : tr("Failed");
            };

            switch (static_cast<Section>(index.column())) {
                case Section::CalibrationResult:
                    return statusToString(item &
                                          RemoteStatus::CamCalibrationSent);
                case Section::UndistortRectifyMap:
                    return statusToString(
                        item & RemoteStatus::UndistortRectifyMapSent);
                case Section::UndistortRectifyMapBin:
                    return statusToString(
                        item & RemoteStatus::UndistortRectifyMapBinGenerated);
                case Section::ToCloud:
                    return tr("NA");
                    return statusToString(item & RemoteStatus::CloudUploaded);
                default:
                    break;
            }
        } break;
        case Qt::DecorationRole: {
            auto statusToIcon = [](bool success) {
                return success ? QIcon{":/icons/16x16/remove-completed.png"}
                               : QIcon{":/icons/16x16/play-stop.png"};
            };

            switch (static_cast<Section>(index.column())) {
                case Section::CalibrationResult:
                    return statusToIcon(item &
                                        RemoteStatus::CamCalibrationSent);
                case Section::UndistortRectifyMap:
                    return statusToIcon(item &
                                        RemoteStatus::UndistortRectifyMapSent);
                case Section::UndistortRectifyMapBin:
                    return statusToIcon(
                        item & RemoteStatus::UndistortRectifyMapBinGenerated);
                case Section::ToCloud:
                    return QIcon{":/icons/16x16/play-idle.png"};
                    return statusToIcon(item & RemoteStatus::CloudUploaded);
                default:
                    break;
            }
        } break;
        case Qt::BackgroundRole: {
            if (index.column() == static_cast<int>(Section::ToCloud)) {
                return QColor{Qt::lightGray};
            }
        } break;
        case Qt::TextAlignmentRole:
            return Qt::AlignCenter;
        default:
            break;
    }

    return {};
}

void RemoteStatusModel::show(const Data &data)
{
    m_items = data;
    resetModel();
}

///------- StereoModuleCalibrationViewPrivate starts from here
class StereoModuleCalibrationViewPrivate
{
    AX_Q_DEFINE_PIMPL(StereoModuleCalibrationView)

public:
    explicit StereoModuleCalibrationViewPrivate(StereoModuleCalibrationView *q);
    ~StereoModuleCalibrationViewPrivate();

    void init();
    void updateUiByDevMode(bool on);

    /// Actions
    void refreshDeviceList();
    void refreshDeviceListUntilAllConnected();
    // In case these methods are bound as slot
    void startOrStopStreaming(bool start);
    inline void startStreaming() { startOrStopStreaming(true); }
    inline void stopStreaming() { startOrStopStreaming(false); }
    void startOrStopRecording(bool start);
    inline void startRecording() { startOrStopRecording(true); }
    inline void stopRecording() { startOrStopRecording(false); }
    // NOTE: Stop is not working
    void startOrStopCalibrate(bool start);
    inline void startCalibration() { startOrStopCalibrate(true); }
    inline void stopCalibration() { startOrStopCalibrate(false); }
    void verifyResults();
    void uploadResults();

    /// Slots
    void handleStereoImageData(int index);
    void handleStereoImageData(const StereoImageData &data, int index);
    void handleImuData(const ImuData &data, int index);
    void handleImuDataList(const ImuDataList &data, int index);
    void onPrepareTaskFinished();
    void onCaptureStateChanged(bool on);
    void onPreprocessingFinished();
    void onCalibrationFinished();

    /// Utils
    void showMessage(const QString &msg, bool clear = true);
    inline void appendMessage(const QString &msg) { showMessage(msg, false); }
    void showImage(const QImage &left, int index);
    void showStereoImages(const QImage &left, const QImage &right, int index);
    void updateLedIndicators(LedIndicator::Status status,
                             const QColor &color = {});

    void showRemoteStatuses(
        const std::vector<RemoteStatusModel::RemoteStatus> &statuses);
    enum Page
    {
        MessagePage,
        RemoteResultPage,
    };
    void showPage(Page page);

public:
    Ui::StereoModuleCalibrationView *ui;
    RemoteStatusModel *m_remoteResultModel;
    StereoModuleHubService *m_hubService;
    CalibTaskService *m_taskService;
    StereoImageDataStream m_stereoStreams;
    StereoModuleTask::ResultReference m_ref;
    QList<QLineEdit *> m_serialNumbers; // Not owned
    QRegularExpression m_snRegex;
    bool m_record{false};
    bool m_devMode{false};
};

StereoModuleCalibrationViewPrivate::StereoModuleCalibrationViewPrivate(
    StereoModuleCalibrationView *q)
    : q_ptr(q),
      ui(new Ui::StereoModuleCalibrationView),
      m_hubService(new StereoModuleHubService),
      m_taskService(new CalibTaskService),
      m_remoteResultModel(new RemoteStatusModel(q))
{
}

StereoModuleCalibrationViewPrivate::~StereoModuleCalibrationViewPrivate()
{
    stopStreaming();
    delete ui;
}

void StereoModuleCalibrationViewPrivate::init()
{
    Q_Q(StereoModuleCalibrationView);
    ui->setupUi(q);
    q->setWindowTitle(
        StereoModuleCalibrationView::tr("Stereo Module Calibration"));

    ui->indicators->setLedCount(StereoModuleHubClient::kMaxSupportDevice);
    ui->captureBtn->setEnabled(false);

    ui->uploadResultsView->setModel(m_remoteResultModel);
    ui->uploadResultsView->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    ui->uploadResultsView->horizontalHeader()->setStretchLastSection(true);

    // FIXME: Avoid hardcoded
    m_serialNumbers.clear();
    for (auto w :
         QList{ui->deviceSN1, ui->deviceSN2, ui->deviceSN3, ui->deviceSN4}) {
        w->setFixedHeight(25);
        m_serialNumbers.push_back(w);
    }

    constexpr char kSNPattern[]{R"([S]\d{2}[C]\d{6})"};
    m_snRegex.setPattern(kSNPattern);

    // Focus chain
    for (int i{0}; i < m_serialNumbers.size() - 1; ++i) {
        q->connect(m_serialNumbers[i], &QLineEdit::textEdited, q,
                   [this, i](const QString &str) {
                       if (m_snRegex.match(str).hasMatch()) {
                           m_serialNumbers[i + 1]->setFocus();
                           m_serialNumbers[i + 1]->clear();
                       }
                   });
    }
    q->connect(m_serialNumbers.back(), &QLineEdit::textEdited, q,
               [this](const QString &str) {
                   if (m_snRegex.match(str).hasMatch()) {
                       // Do nothing now
                       qDebug() << "Last device SN.";
                   }
               });

    // BOLD everything
    auto font = q->font();
    font.setPointSize(15);
    font.setBold(true);
    for (auto w : QWidgetList{ui->refreshBtn, ui->captureBtn}) {
        w->setFont(font);
    }

    font.setPointSize(13);
    font.setBold(false);
    ui->msg->setFont(font);

    // References default values
    m_ref.maxRPE = 0.5;
    m_ref.expectedFocalLength = 850.;
    m_ref.focalLengthTolerance = 30.;
    m_ref.expectedPrincipalPointX = 320.;
    m_ref.expectedPrincipalPointY = 220.;
    m_ref.principalPointXTolerance = 5.;
    m_ref.principalPointYTolerance = 5.;
    m_ref.principalPointDiffTolerance = 25.;
    m_ref.expectedBaseline = 0.09;
    m_ref.baselineTolerance = 0.002;
    m_ref.expectedInterCameraRotation = 0.;
    m_ref.interCameraRotationTolerance = 3.;
    m_ref.expectedImuCameraRotation = 90.;
    m_ref.imuCameraRotationTolerance = 3.;
    ui->calibResultView->setReference(m_ref);

    m_hubService->worker()->bindStereoImageDataStream(&m_stereoStreams);

    updateUiByDevMode(m_devMode);
}

void StereoModuleCalibrationViewPrivate::updateUiByDevMode(bool on)
{
    // Views
    ui->views->setTabVisible(ui->views->indexOf(ui->tab3DView), on);
    ui->views->setTabVisible(ui->views->indexOf(ui->tabDetailsView), on);

    // Buttons
    ui->configBtn->setVisible(on);
}

void StereoModuleCalibrationViewPrivate::refreshDeviceList()
{
    Q_Q(StereoModuleCalibrationView);
    auto connectServer = [this, q]() -> int {
        const bool serverReady = m_hubService->worker()->isChannelReady();
        if (serverReady) {
            emit q->requestShowMessage(StereoModuleCalibrationView::tr(
                "Server is ready, searching devices..."));
            return m_hubService->worker()->onlineDeviceCount();
        }

        emit q->requestShowMessage(StereoModuleCalibrationView::tr(
            "Server is not ready, try to reconnect..."));
        const bool reconnected = m_hubService->worker()->tryToConnect();
        if (reconnected) {
            emit q->requestAppendMessage(StereoModuleCalibrationView::tr(
                "Server reconnected, searching devices..."));
            return m_hubService->worker()->onlineDeviceCount();
        }

        emit q->requestAppendMessage(
            StereoModuleCalibrationView::tr("Failed to reconnect server."));
        return -1;
    };

    auto updateTaskByConnection = [this, q](int count) {
        if (count < 1) {
            emit q->requestAppendMessage(
                StereoModuleCalibrationView::tr("No available devices."));
            return;
        }

        emit q->requestAppendMessage(
            StereoModuleCalibrationView::tr("Find %1 available devices.")
                .arg(QString::number(count)));

        // We only can get device SN from GUI
        auto infos = m_hubService->deviceInfos();
        std::vector<StereoModuleInfo> onlineDeviceInfos;
        for (size_t i{0}; i < infos.size(); ++i) {
            if (auto &info = infos[i]; info.connected) {
                info.sn = m_serialNumbers[i]->text().toStdString();
                onlineDeviceInfos.push_back(info);
            }
        }

        emit m_taskService->prepareTask(onlineDeviceInfos);
    };

    auto future = QtConcurrent::run(connectServer).then(updateTaskByConnection);
    future.waitForFinished();
}

void StereoModuleCalibrationViewPrivate::refreshDeviceListUntilAllConnected()
{
    Q_Q(StereoModuleCalibrationView);
    auto searchDevicesUtilSuccess = [this, q]() -> int {
        const bool serverReady = m_hubService->worker()->isChannelReady();
        if (serverReady) {
            emit q->requestShowMessage(StereoModuleCalibrationView::tr(
                "Server is ready, searching devices..."));
            return m_hubService->worker()->onlineDeviceCount();
        }

        emit q->requestShowMessage(StereoModuleCalibrationView::tr(
            "Server is not ready, try to reconnect..."));
        while (!m_hubService->worker()->tryToConnect()) {
            emit q->requestAppendMessage(StereoModuleCalibrationView::tr(
                "Failed to reconnect server, try to reconnect again..."));
        }

        emit q->requestShowMessage(StereoModuleCalibrationView::tr(
            "Server reconnect, searching devices..."));
        while (m_hubService->worker()->onlineDeviceCount() !=
               StereoModuleHubClient::kMaxSupportDevice) {
            emit q->requestAppendMessage(
                StereoModuleCalibrationView::tr(
                    "Not all (%1) the devices are available, refresh again...")
                    .arg(QString::number(
                        StereoModuleHubClient::kMaxSupportDevice)));
        }

        return StereoModuleHubClient::kMaxSupportDevice;
    };

    auto updateTaskByConnection = [this, q](int count) {
        emit q->requestAppendMessage(
            StereoModuleCalibrationView::tr("Find %1 available devices.")
                .arg(QString::number(count)));

        // We only can get device SN from GUI
        // We know all the devices are connected when the program goes here
        auto infos = m_hubService->deviceInfos();
        for (size_t i{0}; i < infos.size(); ++i) {
            auto &info = infos[i];
            info.sn = m_serialNumbers[i]->text().toStdString();
        }

        emit m_taskService->prepareTask(infos);
    };

    auto future = QtConcurrent::run(searchDevicesUtilSuccess)
                      .then(updateTaskByConnection);
}

void StereoModuleCalibrationViewPrivate::startOrStopStreaming(bool start)
{
    if (start) {
        emit m_hubService->startBlockStreaming();
        // emit m_hubService->startAsyncStreaming();
        // emit m_hubService->startCallbackStreaming();
    }
    else {
        emit m_hubService->stopBlockStreaming();
        // emit m_hubService->stopAsyncStreaming();
        // emit m_hubService->stopCallbackStreaming();
    }
}

void StereoModuleCalibrationViewPrivate::startOrStopRecording(bool start)
{
    m_record = start;
}

void StereoModuleCalibrationViewPrivate::startOrStopCalibrate(bool start)
{
    if (start) {
        emit m_taskService->startCalibration();
    }
    else {
        emit m_taskService->stopCalibration();
    }
}

void StereoModuleCalibrationViewPrivate::verifyResults()
{
    std::vector<StereoModuleTask::ResultSummary> summaries;
    for (const auto &task : m_taskService->tasks()) {
        if (!task->readyToVerify()) {
            summaries.push_back({});
            continue;
        }

        const auto summary = task->verifyCalibResults(m_ref);
        summaries.push_back(summary);

        constexpr char kCalibReportFilename[]{"calibration_report.txt"};
        const auto path = fs::path{io::kSaveDataDir} /
                          io::taskDirName(task->uuid()) / kCalibReportFilename;
        {
            std::ofstream fout{path.string()};
            fout << StereoModuleCalibResultView::verifySummaryToReport(m_ref,
                                                                       summary)
                        .toStdString();
        }
    }

    ui->calibResultView->showSummaries(summaries);
}

void StereoModuleCalibrationViewPrivate::uploadResults()
{
    const auto &indices = m_hubService->onlineDeviceIndices();
    const auto &tasks = m_taskService->tasks();
    if (indices.size() != tasks.size()) {
        return;
    }

    Q_Q(StereoModuleCalibrationView);
    // TODO: Async, parallelize here
    using RemoteStatus = RemoteStatusModel::RemoteStatus;
    std::vector remoteStatusus(StereoModuleHubClient::kMaxSupportDevice,
                               RemoteStatus::AllFailed);
    for (size_t i{0}; i < tasks.size(); ++i) {
        const auto &task = tasks[i];
        const auto &index = indices[i];
        const auto displayIndex = index + 1;
        // TODO: Should use pass() here
        if (!task->readyToVerify()) {
            qDebug() << "Failed to send results to device " << displayIndex
                     << ": "
                        "Task is not verified.";
            continue;
        }

        // Here we assume index can't exceed max support count
        auto &status = remoteStatusus[index];

        /// Calibration results
        const auto calibResult = task->toVioConfigFile();

        // Save results to task folder
        const auto &uuid = task->uuid();
        const auto taskDir = fs::path{io::kSaveDataDir} / io::taskDirName(uuid);
        fs::create_directories(taskDir);

        auto makeLocalCalibFilename = [](const std::string &orgin) {
            constexpr char kCalibPrefix[]{"calibration"};
            return std::format("{0}-{1}", kCalibPrefix, orgin);
        };

        {
            std::ofstream fout{
                taskDir / makeLocalCalibFilename(io::kCalibParameterFilename)};
            fout << calibResult;
        }

        // Send results to device
        appendMessage(StereoModuleCalibrationView::tr(
                          "Sending calibration result to device %1.")
                          .arg(QString::number(displayIndex)));
        if (!m_hubService->worker()->sendCameraCalibrationToDevice(calibResult,
                                                                   index)) {
            appendMessage(StereoModuleCalibrationView::tr(
                              "Failed to send calibration result to device %1.")
                              .arg(QString::number(displayIndex)));
            continue;
        }

        status |= RemoteStatus::CamCalibrationSent;
        appendMessage(StereoModuleCalibrationView::tr(
                          "Sent calibration result to device %1.")
                          .arg(QString::number(displayIndex)));

        /// Undistortion map
        const auto [map1, map2] = task->toStereoMaps();

        appendMessage(StereoModuleCalibrationView::tr(
                          "Sending undistort rectify map to device %1.")
                          .arg(QString::number(displayIndex)));
        if (!m_hubService->worker()->sendUndistortRectifyMapToDevice(map1, map2,
                                                                     index)) {
            appendMessage(
                StereoModuleCalibrationView::tr(
                    "Failed to send undistort rectify map to device %1.")
                    .arg(QString::number(displayIndex)));
            continue;
        }

        status |= RemoteStatus::UndistortRectifyMapSent;
        appendMessage(StereoModuleCalibrationView::tr(
                          "Sent undistort rectify map to device %1.")
                          .arg(QString::number(displayIndex)));

        if (m_hubService->worker()->requestGenerateUndistortRectifyMapBin(
                index)) {
            status |= RemoteStatus::UndistortRectifyMapBinGenerated;
            appendMessage(
                StereoModuleCalibrationView::tr(
                    "Undistort rectify map bin file on device %1 generated.")
                    .arg(QString::number(displayIndex)));
        }
        else {
            appendMessage(
                StereoModuleCalibrationView::tr("Failed to generate undistort "
                                                "rectify map bin on device %1.")
                    .arg(QString::number(displayIndex)));
        }
    }

    showRemoteStatuses(remoteStatusus);
}

void StereoModuleCalibrationViewPrivate::handleStereoImageData(int index)
{
    if (index >= m_stereoStreams.size() || index < 0) {
        return;
    }

    auto &stream = m_stereoStreams.at(index);
    if (stream.closed() || stream.empty()) {
        return;
    }

    Q_Q(StereoModuleCalibrationView);
    StereoImageData data;
    stream >> data;

    if (!data.isValid()) {
        return;
    }

    cv::Mat merged;
    cv::hconcat(data.left, data.right, merged);
    emit q->requestShowImage(cvMatToQImage(merged), index);

    if (m_record) {
        const auto pos = m_hubService->devicePosOf(index);
        emit m_taskService->processStereoImageData(data.deepCopy(), pos);
    }
}

void StereoModuleCalibrationViewPrivate::handleStereoImageData(
    const StereoImageData &data, int index)
{
    if (!data.isValid()) {
        return;
    }

    Q_Q(StereoModuleCalibrationView);
    emit q->requestShowStereoImages(cvMatToQImage(data.left),
                                    cvMatToQImage(data.right), index);

    if (m_record) {
        const auto pos = m_hubService->devicePosOf(index);
        emit m_taskService->processStereoImageData(data, pos);
    }
}

void StereoModuleCalibrationViewPrivate::handleImuData(const ImuData &data,
                                                       int index)
{
    if (m_record) {
        const auto pos = m_hubService->devicePosOf(index);
        emit m_taskService->processMotionData(data, pos);
    }
}

void StereoModuleCalibrationViewPrivate::handleImuDataList(
    const ImuDataList &data, int index)
{
    if (m_record) {
        const auto pos = m_hubService->devicePosOf(index);
        emit m_taskService->processMotionDataList(data, pos);
    }
}

void StereoModuleCalibrationViewPrivate::onPrepareTaskFinished()
{
    Q_Q(StereoModuleCalibrationView);
    // GUI
    ui->captureBtn->setEnabled(true);
    ui->calibResultView->showPlaceholder();
    appendMessage(
        StereoModuleCalibrationView::tr("Calibration tasks are ready."));
    if (!m_taskService->saveData()) {
        appendMessage(StereoModuleCalibrationView::tr(
            "Not enough space on the disk, saving data is disabled."));
    }
    updateLedIndicators(LedIndicator::Status::On, Qt::green);
    appendMessage(
        StereoModuleCalibrationView::tr("About to start live streaming..."));
    appendMessage(StereoModuleCalibrationView::tr(
        "Start capturing data when the calibration target "
        "can be clearly visualized in all the views."));

    // Actions
    startStreaming();
}

void StereoModuleCalibrationViewPrivate::onCaptureStateChanged(bool on)
{
    Q_Q(StereoModuleCalibrationView);
    // GUI
    ui->captureBtn->setIcon(QIcon{on ? kStopIconName : kPlayIconName});
    ui->captureBtn->setText(
        on ? StereoModuleCalibrationView::tr("Stop Capture")
           : StereoModuleCalibrationView::tr("Start Capture"));

    if (on) {
        startRecording();
        showMessage(StereoModuleCalibrationView::tr("Start capturing..."));
        updateLedIndicators(LedIndicator::Status::Blink, Qt::green);
    }
    else {
        stopRecording();
        stopStreaming();
        appendMessage(StereoModuleCalibrationView::tr("Capturing stop."));
        appendMessage(StereoModuleCalibrationView::tr(
            "Streaming is about to stop, "
            "data preprocessing may still be running."));
        emit m_taskService->streamingAboutToStop();
    }
}

void StereoModuleCalibrationViewPrivate::onPreprocessingFinished()
{
    Q_Q(StereoModuleCalibrationView);
    appendMessage(StereoModuleCalibrationView::tr("Data preprocessing finished."
                                                  "\n"
                                                  "Start to calibrate..."));
    updateLedIndicators(LedIndicator::Status::Blink, Qt::yellow);

    startCalibration();
}

void StereoModuleCalibrationViewPrivate::onCalibrationFinished()
{
    Q_Q(StereoModuleCalibrationView);
    appendMessage(StereoModuleCalibrationView::tr("Calculation finished."));
    appendMessage(StereoModuleCalibrationView::tr(
        "Start to verify calibration results."));

    verifyResults();

    appendMessage(StereoModuleCalibrationView::tr(
        "Verification finished. Please check the results below."));

    // Update LED indicators
    const auto &tasks = m_taskService->tasks();
    std::vector ledColors(StereoModuleHubClient::kMaxSupportDevice,
                          QColor{Qt::green});
    std::vector ledStatuses(StereoModuleHubClient::kMaxSupportDevice,
                            LedIndicator::Status::Off);
    for (size_t i{0}; i < tasks.size(); ++i) {
        const auto index = m_hubService->deviceIndexAt(i);
        if (index < 0) {
            continue;
        }

        ledColors[index] = tasks[i]->pass() ? Qt::green : Qt::red;
        ledStatuses[index] = LedIndicator::Status::Blink;
    }
    ui->indicators->setLedColors(ledColors);
    ui->indicators->setLedStatuses(ledStatuses);

    showMessage(StereoModuleCalibrationView::tr(
        "Start to upload calibration result to devices."));

    uploadResults();

    showPage(Page::RemoteResultPage);
    ui->calibResultView->hidePlaceholder();
    appendMessage(StereoModuleCalibrationView::tr(
        "Upload finished. Please check upload status below."));
    appendMessage(
        StereoModuleCalibrationView::tr("Remove devices from the dock."));
}

void StereoModuleCalibrationViewPrivate::showMessage(const QString &msg,
                                                     bool clear)
{
    if (clear) {
        ui->msg->clear();
    }
    ui->msg->message(msg);
}

void StereoModuleCalibrationViewPrivate::showImage(const QImage &image,
                                                   int index)
{
    if (image.isNull()) {
        return;
    }

    ui->liveView->showImage(image, index);
}

void StereoModuleCalibrationViewPrivate::showStereoImages(const QImage &left,
                                                          const QImage &right,
                                                          int index)
{
    const auto merge = qimg::hconcat(left, right);
    if (merge.isNull()) {
        return;
    }

    ui->liveView->showImage(merge, index);
}

void StereoModuleCalibrationViewPrivate::updateLedIndicators(
    LedIndicator::Status status, const QColor &color)
{
    // We are sured that indices
    // 1. Start from 0
    // 2. In ascending order
    const auto &indices = m_hubService->onlineDeviceIndices();
    if (indices.size() > StereoModuleHubClient::kMaxSupportDevice) {
        return;
    }

    std::vector statuses(StereoModuleHubClient::kMaxSupportDevice,
                         LedIndicator::Status::Off);
    for (const auto &index : indices) {
        statuses[index] = status;
    }
    ui->indicators->setLedStatuses(statuses);

    if (!color.isValid()) {
        return;
    }

    std::vector colors(StereoModuleHubClient::kMaxSupportDevice,
                       ui->indicators->ledOnColor());
    for (const auto &index : indices) {
        colors[index] = color;
    }

    ui->indicators->setLedColors(colors);
}

void StereoModuleCalibrationViewPrivate::showRemoteStatuses(
    const std::vector<RemoteStatusModel::RemoteStatus> &statuses)
{
    m_remoteResultModel->show(statuses);
}

void StereoModuleCalibrationViewPrivate::showPage(Page page)
{
    switch (page) {
        case Page::MessagePage:
            ui->pages->setCurrentIndex(ui->pages->indexOf(ui->pageMsg));
            break;
        case Page::RemoteResultPage:
            ui->pages->setCurrentIndex(
                ui->pages->indexOf(ui->pageUploadResults));
            break;
        default:
            break;
    }
}

///------- StereoModuleCalibrationView starts from here
StereoModuleCalibrationView::StereoModuleCalibrationView(QWidget *parent)
    : QWidget(parent), d_ptr(new StereoModuleCalibrationViewPrivate(this))
{
    Q_D(StereoModuleCalibrationView);
    d->init();

    /// Async call
    connect(this, &StereoModuleCalibrationView::requestShowMessage, this,
            [d](const QString &msg) { d->showMessage(msg); });
    connect(this, &StereoModuleCalibrationView::requestAppendMessage, this,
            [d](const QString &msg) { d->appendMessage(msg); });
    connect(
        this, &StereoModuleCalibrationView::requestShowImage, this,
        [d](const QImage &image, int index) { d->showImage(image, index); });
    connect(this, &StereoModuleCalibrationView::requestShowStereoImages, this,
            [d](const QImage &left, const QImage &right, int index) {
                d->showStereoImages(left, right, index);
            });

    /// GUI response
    connect(d->ui->refreshBtn, &QAbstractButton::clicked, this, [d]() {
        d->showPage(StereoModuleCalibrationViewPrivate::Page::MessagePage);
        d->refreshDeviceList();
    });
    connect(d->ui->captureBtn, &QAbstractButton::toggled, this,
            [d](bool checked) { d->onCaptureStateChanged(checked); });
    connect(d->ui->configBtn, &QAbstractButton::clicked, this, [this, d]() {
        StereoModuleCalibrationConfigDialog configs{this};
        configs.widget()->setDefaultValues(d->m_ref);
        if (!configs.exec()) {
            return;
        }

        d->m_ref = configs.widget()->resultReference();
        d->ui->calibResultView->setReference(d->m_ref);
    });

    /// Service response
    connect(d->m_hubService, &StereoModuleHubService::streamStereoFinished,
            this, [d](int index, bool withError) {
                d->appendMessage(
                    tr("Streaming stereo data finish %1 at dock %2.")
                        .arg(withError ? tr("with error") : tr("with no error"),
                             QString::number(index + 1)));
                if (withError) {
                    d->ui->indicators->setLed(index, LedIndicator::Status::On,
                                              Qt::red);
                }
            });
    connect(d->m_hubService, &StereoModuleHubService::streamImuFinished, this,
            [d](int index, bool withError) {
                d->appendMessage(
                    tr("Streaming IMU data finish %1 at dock %2.")
                        .arg(withError ? tr("with error") : tr("with no error"),
                             QString::number(index + 1)));
                if (withError) {
                    d->ui->indicators->setLed(index, LedIndicator::Status::On,
                                              Qt::red);
                }
            });
    connect(d->m_hubService,
            &StereoModuleHubService::stereoImageDataBufferUpdated, this,
            [d](int index) { d->handleStereoImageData(index); });
    connect(d->m_hubService, &StereoModuleHubService::stereoImageDataReceived,
            this, [d](const StereoImageData &data, int index) {
                d->handleStereoImageData(data, index);
            });
    connect(
        d->m_hubService, &StereoModuleHubService::imuDataReceived, this,
        [d](const ImuData &data, int index) { d->handleImuData(data, index); });
    connect(d->m_hubService, &StereoModuleHubService::imuDataListReceived, this,
            [d](const std::vector<ImuData> &data, int index) {
                d->handleImuDataList(data, index);
            });

    connect(d->m_taskService, &CalibTaskService::prepareTaskFinished, this,
            [d]() { d->onPrepareTaskFinished(); });
    connect(d->m_taskService, &CalibTaskService::preprocessingFinished, this,
            [d]() { d->onPreprocessingFinished(); });
    connect(d->m_taskService, &CalibTaskService::allCalibrationFinished, this,
            [d]() { d->onCalibrationFinished(); });
}

StereoModuleCalibrationView::~StereoModuleCalibrationView() = default;

bool StereoModuleCalibrationView::setFromJson(const QJsonObject &jo)
{
    // TODO
    return false;
}

void StereoModuleCalibrationView::setDevMode(bool on)
{
    Q_D(StereoModuleCalibrationView);
    if (d->m_devMode == on) {
        return;
    }

    d->m_devMode = on;
    d->updateUiByDevMode(on);
}

} // namespace tl

#include "StereoModuleCalibrationView.moc"
#include "moc_StereoModuleCalibrationView.cpp"
