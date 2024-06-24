#include <filesystem>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <csv-parser/csv.hpp>

#include <tCore/TimeUtils>
#include <tCalibration/ImuIntrinsicsCalibration>
#include <tMotion/ImuData>

namespace fs = std::filesystem;
using namespace tl;

constexpr char kSequence[]{"seq"};
constexpr char kTimestamp[]{"timestamp"};
constexpr char kAccX[]{"acc_x"};
constexpr char kAccY[]{"acc_y"};
constexpr char kAccZ[]{"acc_z"};
constexpr char kGyroX[]{"gyro_x"};
constexpr char kGyroY[]{"gyro_y"};
constexpr char kGyroZ[]{"gyro_z"};

ImuReadings loadMotionDataFromMat(const std::string &filename)
{
    ImuReadings samples;

    std::string line;
    std::ifstream infile;
    double ts, d[3];

    infile.open(filename);
    if (infile.is_open()) {
        char format[266];
        sprintf(format, "%%lf %%lf %%lf %%lf");

        int l = 0;
        while (getline(infile, line)) {
            int res = sscanf(line.data(), format, &ts, &d[0], &d[1], &d[2]);
            if (res != 4) {
                std::cout << "importAsciiData(): error importing data in line "
                          << l << ", exit" << std::endl;
            }
            else {
                samples.d().push_back(ImuReading(ts, d[0], d[1], d[2]));
            }
            l++;
        }
        infile.close();
    }

    return samples;
}

inline ImuDatas loadMotionData(const std::string &filepath)
{
    ImuDatas imu;
    csv::CSVReader reader{filepath};
    for (auto &row : reader) {
        auto timestamp = time::nsToS(row[kTimestamp].get<uint64_t>());
        imu.acc.d().emplace_back(timestamp, row[kAccX].get<double>(),
                                 row[kAccY].get<double>(),
                                 row[kAccZ].get<double>());
        imu.gyro.d().emplace_back(timestamp, row[kGyroX].get<double>(),
                                  row[kGyroY].get<double>(),
                                  row[kGyroZ].get<double>());
        imu.timeline.emplace_back(timestamp);
    }

    return imu;
}

TEST(IMU, Dataset1)
{
    const std::string dataset_path{
        "/home/bobblelaw/Data/imu_intrinsics_trial1.csv"};
    const auto imu_data = loadMotionData(dataset_path);

    ImuIntrinsicsCalibration imuCalib;

    auto &opts = imuCalib.rOptions();
    opts.gravity = 9.81107;
    opts.initStaticDuration = 50.;
    opts.minStaticIntervalCount = 5;

    imuCalib.calibAccelerometerGyroscope(imu_data.acc, imu_data.gyro);

    const auto &acc_intri = imuCalib.acclIntrinsics();
    const auto &gyr_intri = imuCalib.gyroIntrinsics();

    LOG(INFO) << "Accelerometer misalignment: "
              << acc_intri.misalignmentMatrix();
    LOG(INFO) << "Accelerometer scale: " << acc_intri.scaleMatrix();
    LOG(INFO) << "Accelerometer bias: " << acc_intri.bias();
    LOG(INFO) << "Gyroscope scale: " << gyr_intri.misalignmentMatrix();
    LOG(INFO) << "Gyroscope misalignment: " << gyr_intri.scaleMatrix();
    LOG(INFO) << "Gyroscope bias: " << gyr_intri.bias();

    EXPECT_TRUE(true);
}

TEST(IMU, Dataset_imutk)
{
    const auto acc =
        loadMotionDataFromMat("/home/bobblelaw/Data/xsens_acc.mat");
    const auto gyr =
        loadMotionDataFromMat("/home/bobblelaw/Data/xsens_gyro.mat");

    ImuIntrinsicsCalibration imuCalib;

    auto &opts = imuCalib.rOptions();
    opts.gravity = 9.81107;
    //    opts.initStaticDuration = 50.;
    opts.minStaticIntervalCount = 5;
    opts.useMeanAcc = false;

    ImuIntrinsics acc_intri_init;
    acc_intri_init.setBias({32768., 32768., 32768.});
    ImuIntrinsics gyr_intri_init;
    gyr_intri_init.setScale({1. / 6258., 1. / 6258., 1. / 6258.});
    imuCalib.setInitAcclIntrinsics(acc_intri_init);
    imuCalib.setInitGyroIntrinsics(gyr_intri_init);

    imuCalib.calibAccelerometerGyroscope(acc, gyr);

    const auto &acc_intri = imuCalib.acclIntrinsics();
    const auto &gyr_intri = imuCalib.gyroIntrinsics();

    LOG(INFO) << "Accelerometer misalignment: "
              << acc_intri.misalignmentMatrix();
    LOG(INFO) << "Accelerometer scale: " << acc_intri.scaleMatrix();
    LOG(INFO) << "Accelerometer bias: " << acc_intri.bias();
    LOG(INFO) << "Gyroscope scale: " << gyr_intri.misalignmentMatrix();
    LOG(INFO) << "Gyroscope misalignment: " << gyr_intri.scaleMatrix();
    LOG(INFO) << "Gyroscope bias: " << gyr_intri.bias();

    EXPECT_TRUE(true);
}
