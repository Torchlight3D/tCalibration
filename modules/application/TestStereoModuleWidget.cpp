#include "TestStereoModuleWidget.h"
#include "ui_TestStereoModuleWidget.h"

#include <filesystem>

#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/imgproc.hpp>
#include <QFileDialog>
#include <QtConcurrent/QtConcurrent>

// TEST:
#include <opencv2/imgcodecs.hpp>
#include <csv-parser/csv.hpp>
#include <AxCore/TimeUtils>

#include <AxCamera/OmnidirectionalCameraModel>
// #include <AxDevice/StereoDataPlayer>
#include <AxImgproc/BlurDetection>

#include "gui/QtOpenCV.h"
#include "StereoModuleData.h"
#include "StereoModuleTask.h"

namespace fs = std::filesystem;

namespace thoht {

///------- StereoDataPlayer starts from here
namespace {
constexpr char kMotionFilename[]{"motion.csv"};
}

namespace key {
constexpr char kSequence[]{"seq"};
constexpr char kTimestamp[]{"timestamp"};
constexpr char kAccX[]{"acc_x"};
constexpr char kAccY[]{"acc_y"};
constexpr char kAccZ[]{"acc_z"};
constexpr char kGyroX[]{"gyro_x"};
constexpr char kGyroY[]{"gyro_y"};
constexpr char kGyroZ[]{"gyro_z"};
} // namespace key

// Assume no one changes (delete/add/modify) the contents during operation
class StereoDataPlayer
{
public:
    explicit StereoDataPlayer(const std::string &rootPath);

    bool isValid() const;

    void setStep(int step);

    void toNext();
    void toPrev();
    void toBegin();
    void toEnd();
    void toIndex(qsizetype index);
    qsizetype imageCount() const;

    cv::Mat currentLeft() const;
    cv::Mat currentRight() const;
    double currentTimestamp() const;

    const auto &imuDatas() const { return imu_datas_; }

private:
    bool validatePath(const std::string &path);
    void loadImuData();

private:
    QString root_;
    QStringList filenames_;
    qsizetype index_;
    int step_;
    ImuDatas imu_datas_;
};

StereoDataPlayer::StereoDataPlayer(const std::string &path) : step_(1)
{
    if (validatePath(path)) {
        loadImuData();
    }

    qDebug() << "Load IMU data size: " << imu_datas_.size();
    qDebug() << "Load image data size: " << filenames_.size();
}

bool StereoDataPlayer::isValid() const
{
    return !filenames_.empty() && !root_.isEmpty();
}

void StereoDataPlayer::setStep(int step)
{
    if (step > 0) {
        step_ = step;
    }
}

void StereoDataPlayer::toNext()
{
    index_ = std::min(filenames_.size() - 1, index_ + step_);
}

void StereoDataPlayer::toPrev() { index_ = std::max(0ll, index_ - step_); }

void StereoDataPlayer::toBegin() { index_ = 0; }

void StereoDataPlayer::toEnd() { index_ = filenames_.size() - 1; }

void StereoDataPlayer::toIndex(qsizetype index)
{
    index_ = std::clamp(index, 0ll, filenames_.size() - 1);
}

qsizetype StereoDataPlayer::imageCount() const { return filenames_.size(); }

cv::Mat StereoDataPlayer::currentLeft() const
{
    const auto leftPath = QDir{root_}.filesystemPath() / "left" /
                          filenames_[index_].toStdString();
    const auto image = cv::imread(leftPath.string());
    return image;
}

cv::Mat StereoDataPlayer::currentRight() const
{
    const auto rightPath = QDir{root_}.filesystemPath() / "right" /
                           filenames_[index_].toStdString();
    const auto image = cv::imread(rightPath.string());
    return image;
}

double StereoDataPlayer::currentTimestamp() const
{
    const auto &filename = filenames_[index_];
    return time::nsToS(filename.left(filename.lastIndexOf('.')).toLong());
}

bool StereoDataPlayer::validatePath(const std::string &path)
{
    const auto datapath = QString::fromStdString(path);
    QDir dataset{datapath};
    if (!dataset.exists()) {
        return false;
    }

    if (!dataset.exists(kMotionFilename)) {
        return false;
    }

    const bool leftExist = dataset.cd("left");
    if (!leftExist) {
        return false;
    }

    const auto leftFileNames =
        dataset.entryList({"*.png", "*.jpg"}, QDir::Files, QDir::Name);

    dataset.cdUp();

    const bool rightExist = dataset.cd("right");
    if (!rightExist) {
        return false;
    }

    const auto rightFileNames =
        dataset.entryList({"*.png", "*.jpg"}, QDir::Files, QDir::Name);

    if (leftFileNames.size() != rightFileNames.size()) {
        return false;
    }

    root_ = datapath;
    filenames_ = leftFileNames;
    index_ = 0;

    // TEST
    for (const auto &filename :
         std::vector(filenames_.begin(), filenames_.begin() + 15)) {
        LOG(INFO) << "Filename: " << filename.toStdString();
    }

    return true;
}

void StereoDataPlayer::loadImuData()
{
    csv::CSVReader reader{
        QDir{root_}.absoluteFilePath(kMotionFilename).toStdString()};
    for (auto &row : reader) {
        auto timestamp = time::nsToS(row[key::kTimestamp].get<long>());
        imu_datas_.acc.d().emplace_back(
            timestamp, row[key::kAccX].get<double>(),
            row[key::kAccY].get<double>(), row[key::kAccZ].get<double>());
        imu_datas_.gyro.d().emplace_back(
            timestamp, row[key::kGyroX].get<double>(),
            row[key::kGyroY].get<double>(), row[key::kGyroZ].get<double>());
        imu_datas_.timeline.emplace_back(timestamp);
    }
}

///------- TestStereoModuleWidgetPrivate starts from here
class TestStereoModuleWidgetPrivate
{
    Q_DISABLE_COPY(TestStereoModuleWidgetPrivate)
    Q_DECLARE_PUBLIC(TestStereoModuleWidget)
    TestStereoModuleWidget *const q_ptr;

public:
    explicit TestStereoModuleWidgetPrivate(TestStereoModuleWidget *q);
    ~TestStereoModuleWidgetPrivate();

    void init();

    /// Interfaces
    void openSettings();
    void openDataset();
    void startOrStopSimulateStreaming(bool start);
    void testCalibration();
    void testRectification();
    void toPrev();
    void toNext();

    /// Other algos
    void detectCurrentBlur();

    /// Utils
    void showStereo(const QImage &left, const QImage &right);

public:
    Ui::TestStereoModuleWidget *ui;
    std::unique_ptr<StereoDataPlayer> m_player;
    std::unique_ptr<StereoModuleTask> m_task;
};

TestStereoModuleWidgetPrivate::TestStereoModuleWidgetPrivate(
    TestStereoModuleWidget *q)
    : q_ptr(q), ui(new Ui::TestStereoModuleWidget)
{
}

TestStereoModuleWidgetPrivate::~TestStereoModuleWidgetPrivate() { delete ui; }

void TestStereoModuleWidgetPrivate::init()
{
    Q_Q(TestStereoModuleWidget);
    ui->setupUi(q);
}

void TestStereoModuleWidgetPrivate::openSettings()
{
    // TODO: Open Settings dialog
    StereoModuleTask::Options options;
    options.skipStep = 15;
    options.maxIntrinsicsViewCount = 50;

    m_task.reset(new StereoModuleTask(options));

    m_task->prepare();
}

void TestStereoModuleWidgetPrivate::openDataset()
{
    Q_Q(TestStereoModuleWidget);
    const auto path = QFileDialog::getExistingDirectory(q, "Choose dataset...",
                                                        QDir::homePath());
    if (path.isEmpty()) {
        return;
    }

    m_player.reset(new StereoDataPlayer(path.toStdString()));
    if (!m_player->isValid()) {
        qWarning() << "Invalid dataset path: " << path;
    }
    else {
        ui->dataPath->setText(path);
    }
}

void TestStereoModuleWidgetPrivate::startOrStopSimulateStreaming(bool start)
{
    if (!m_player->isValid()) {
        qWarning() << "No valid dataset.";
        return;
    }

    // FIXME: Cancel and suspend wont work
    if (start) {
        // Animate widget
        ui->loadDataBtn->setText("Cancel");
        ui->progressBar->setRange(0, 0);

        // Do work
        const auto _ =
            QtConcurrent::run([this]() {
                const int imgCount =
                    std::min(1000, int(m_player->imageCount()));
                for (int i{0}; i < imgCount; ++i) {
                    const StereoImageData stereo{m_player->currentLeft(),
                                                 m_player->currentRight(),
                                                 m_player->currentTimestamp()};
                    m_task->addStereoData(stereo);
                    m_player->toNext();
                }
            }).then([this]() {
                m_task->setMotionData(m_player->imuDatas());

                // Update ui
                //            ui->progressBar->setRange(0, 100);
                //            ui->progressBar->setValue(100);
            });
    }
    else {
        ui->progressBar->setRange(0, 100);
        ui->progressBar->setValue(100);

        ui->loadDataBtn->setText("Load All");
    }
}

void TestStereoModuleWidgetPrivate::testCalibration()
{
    Q_Q(TestStereoModuleWidget);
    ui->calibBtn->setEnabled(false);
    ui->progressBar->setRange(0, 0);

    const auto _ =
        QtConcurrent::run([this]() {
            m_task->startCalculation();
            std::string calib_res = m_task->toLubaVioResult();
            {
                std::ofstream fout("./calib_parameters.yaml", std::ios::out);
                fout << calib_res;
            }
        }).then([this]() {
            // Update ui
            //        ui->calibBtn->setEnabled(true);
            //        ui->progressBar->setRange(0, 100);
            //        ui->progressBar->setValue(100);
        });
}

void TestStereoModuleWidgetPrivate::testRectification()
{
    OmnidirectionalCameraModel cameraLeft_kalibr;
    cameraLeft_kalibr.setFocalLength(837.4664892993342);
    cameraLeft_kalibr.setAspectRatio(0.997876);
    cameraLeft_kalibr.setPrincipalPoint(332.00617987478984, 191.08042516943507);
    cameraLeft_kalibr.setMirrorDistortion(1.60826862964);
    cameraLeft_kalibr.setRadialDistortion(-0.05752771998045863,
                                          0.22237550975545115);
    cameraLeft_kalibr.setTangentialDistortion(0.0018744794789161787,
                                              1.2160264129479937e-05);

    OmnidirectionalCameraModel cameraRight_kalibr;
    cameraRight_kalibr.setFocalLength(843.8702964681291);
    cameraRight_kalibr.setAspectRatio(0.997671);
    cameraRight_kalibr.setPrincipalPoint(328.1988291811307, 220.10441398091385);
    cameraRight_kalibr.setMirrorDistortion(1.62029488584);
    cameraRight_kalibr.setRadialDistortion(-0.05934772156529351,
                                           0.21148118746174419);
    cameraRight_kalibr.setTangentialDistortion(0.0033791017770635336,
                                               -0.00026427271497360904);

    // clang-format off
    cv::Mat R = (cv::Mat_<double>(3, 3)
                     << 0.99999322, -0.00353242, -0.00103666,
                 0.00353404,  0.99999253,  0.00156384,
                 0.00103113, -0.00156749,  0.99999824);
    // clang-format on
    cv::Mat t = (cv::Mat_<double>(1, 3) << -0.0900135, -0.00046536, 0.00101718);

    cv::Mat R0, R1;
    cv::omnidir::stereoRectify(R, t, R0, R1);

    const cv::Size imgSize{640, 480};
    const int flags = cv::omnidir::RECTIFY_PERSPECTIVE;
    cv::Matx33f Knew{imgSize.width / 3.1415f,
                     0.f,
                     0.f,
                     0.f,
                     imgSize.height / 3.1415f,
                     0.f,
                     0.f,
                     0.f,
                     1.f};
    //    cv::Matx33f Knew(imgSize.width / 4, 0, imgSize.width / 2, 0,
    //                     imgSize.height / 4, imgSize.height / 2, 0, 0, 1);

    // Left
    // Kalibr
    //    cv::Mat K0 =
    //        (cv::Mat_<double>(3, 3) << 837.4664892993342, 0.,
    //        332.00617987478984,
    //         0., 835.6876065352099, 191.08042516943507, 0., 0., 1.);
    //    cv::Mat D0 =
    //        (cv::Mat_<double>(1, 4) << -0.05752771998045863,
    //        0.22237550975545115,
    //         0.0018744794789161787, 1.2160264129479937e-05);
    //    cv::Mat xi0 = (cv::Mat_<double>(1, 1) << 1.60826862964);

    // CalibKit
    cv::Mat K0 = (cv::Mat_<double>(3, 3) << 6462.95, 0., 332.609, 0., 6451.27,
                  190.181, 0., 0., 1.);
    cv::Mat D0 =
        (cv::Mat_<double>(1, 4) << 51.3726, 22729.6, 0.0261727, -0.00719056);
    cv::Mat xi0 = (cv::Mat_<double>(1, 1) << 19.0135);

    cv::Mat P0 = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat mapX0, mapY0;
    cv::omnidir::initUndistortRectifyMap(K0, D0, xi0, R0, Knew, imgSize,
                                         CV_32FC1, mapX0, mapY0, flags);
    //    initUndistortRectifyMap(cameraLeft_kalibr, R0, imgSize, mapX0, mapY0);

    // Right
    cv::Mat K1 =
        (cv::Mat_<double>(3, 3) << 843.8702964681291, 0., 328.1988291811307, 0.,
         841.9048307496272, 220.10441398091385, 0., 0., 1.);
    cv::Mat D1 =
        (cv::Mat_<double>(1, 4) << -0.05934772156529351, 0.21148118746174419,
         0.0033791017770635336, -0.00026427271497360904);
    cv::Mat xi1 = (cv::Mat_<double>(1, 1) << 1.62029488584);
    cv::Mat P1 = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat mapX1, mapY1;
    cv::omnidir::initUndistortRectifyMap(K1, D1, xi1, R1, Knew, imgSize,
                                         CV_32FC1, mapX1, mapY1, flags);
    //    initUndistortRectifyMap(cameraRight_kalibr, R1, imgSize, mapX1,
    //    mapY1);

    Q_Q(TestStereoModuleWidget);
    cv::Mat imgLeftUndistort;
    cv::remap(m_player->currentLeft(), imgLeftUndistort, mapX0, mapY0,
              cv::INTER_NEAREST);
    //    cv::omnidir::undistortImage(d->m_player->currentLeft(),
    //    imgLeftUndistort,
    //                                K0, D0, xi0, flags, Knew, imgSize);

    constexpr int kLines = 20;
    const int step = imgSize.height / 20;
    for (int i{0}; i * step <= imgSize.height; ++i) {
        cv::line(imgLeftUndistort, cv::Point(0, i * step),
                 cv::Point(imgSize.width, i * step), CV_RGB(0, 255, 0));
    }

    cv::Mat imgRightUndistort;
    cv::remap(m_player->currentRight(), imgRightUndistort, mapX1, mapY1,
              cv::INTER_NEAREST);
    cv::omnidir::undistortImage(m_player->currentRight(), imgRightUndistort, K1,
                                D1, xi1, flags, Knew, imgSize);

    for (int i{0}; i * step <= imgSize.height; ++i) {
        cv::line(imgRightUndistort, cv::Point(0, i * step),
                 cv::Point(imgSize.width, i * step), CV_RGB(0, 255, 0));
    }

    showStereo(cvMatToQImage(imgLeftUndistort),
               cvMatToQImage(imgRightUndistort));

    m_player->toNext();
}

void TestStereoModuleWidgetPrivate::toPrev() { m_player->toPrev(); }

void TestStereoModuleWidgetPrivate::toNext() { m_player->toNext(); }

void TestStereoModuleWidgetPrivate::detectCurrentBlur()
{
    LaplacianModifiedBlurDetection detector;

    auto left = m_player->currentLeft();
    auto right = m_player->currentRight();
    qDebug() << "Left blur: " << detector.evaluate(left);
    qDebug() << "Right blur: " << detector.evaluate(right);
    showStereo(cvMatToQImage(left), cvMatToQImage(right));
}

void TestStereoModuleWidgetPrivate::showStereo(const QImage &left,
                                               const QImage &right)
{
    ui->leftImg->showImage(left);
    ui->rightImg->showImage(right);
}

///------- TestStereoModuleWidget starts from here
TestStereoModuleWidget::TestStereoModuleWidget(QWidget *parent)
    : QWidget(parent), d_ptr(new TestStereoModuleWidgetPrivate(this))
{
    Q_D(TestStereoModuleWidget);
    d->init();

    connect(d->ui->configBtn, &QAbstractButton::clicked, this,
            [d]() { d->openSettings(); });
    connect(d->ui->openDataBtn, &QAbstractButton::clicked, this,
            [d]() { d->openDataset(); });
    connect(d->ui->loadDataBtn, &QAbstractButton::toggled, this,
            [d](bool checked) { d->startOrStopSimulateStreaming(checked); });
    connect(d->ui->calibBtn, &QAbstractButton::clicked, this,
            [d]() { d->testCalibration(); });
    connect(d->ui->rectifyBtn, &QAbstractButton::clicked, this,
            [d]() { d->testRectification(); });
}

TestStereoModuleWidget::~TestStereoModuleWidget() = default;

} // namespace thoht

#include "moc_TestStereoModuleWidget.cpp"
