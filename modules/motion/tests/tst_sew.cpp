#include <fstream>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <tCore/TimeUtils>
#include <tMotion/ImuData>
#include <tMotion/Spline/SplineErrorWeighting>

using namespace tl;
using namespace nlohmann; // json

namespace {
constexpr char kAccelerometer[]{"accelerometer"};
constexpr char kGyroscope[]{"gyroscope"};
constexpr char kTimestamp[]{"timestamps_ns"};
} // namespace

bool readImuData(const std::string& filename, ImuDatas& datas)
{
    std::ifstream file;
    file.open(filename.c_str());
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file";
        return false;
    }

    json j;
    file >> j;

    const auto accl = j[kAccelerometer];
    const auto gyro = j[kGyroscope];
    const auto timestamp = j[kTimestamp];

    const auto n_sample = timestamp.size();
    if (timestamp.empty() || gyro.size() != n_sample ||
        accl.size() != n_sample) {
        LOG(ERROR) << "Invalid data format.";
        return false;
    }

    datas.reserve(n_sample);
    for (size_t i{0}; i < n_sample; ++i) {
        auto t = time::nsToS(timestamp[i]);
        datas.acc.d().emplace_back(t, accl[i][0], accl[i][1], accl[i][2]);
        datas.gyro.d().emplace_back(t, gyro[i][0], gyro[i][1], gyro[i][2]);
        datas.timeline.emplace_back(t);
    }

    return true;
}

TEST(Calibration, SEW)
{
    ImuDatas datas;
    EXPECT_TRUE(readImuData("imu.json", datas));

    double quality{0.99};
    double accMinSpacing{0.01}, accMaxSpacing{0.2};
    double accSpacing, accVar;
    SplineErrorWeighting::estKnotSpacingAndVariance(
        datas.acc, quality, accMinSpacing, accMaxSpacing, accSpacing, accVar);
    double accFactor = 1. / std::sqrt(accVar);
    EXPECT_NEAR(accSpacing, 0.09, 1e-3);
    EXPECT_NEAR(accFactor, 3.852, 1e-3);

    double gyrMinSpacing{0.01}, gyrMaxSpacing{0.15};
    double gyrSpacing, gyrVar;
    SplineErrorWeighting::estKnotSpacingAndVariance(
        datas.gyro, quality, gyrMinSpacing, gyrMaxSpacing, gyrSpacing, gyrVar);
    double gyrFactor = 1. / std::sqrt(gyrVar);
    EXPECT_NEAR(gyrSpacing, 0.05, 1e-3);
    EXPECT_NEAR(gyrFactor, 22.374, 1e-3);
}
