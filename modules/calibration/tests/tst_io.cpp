#include <fstream>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCalibration/CalibrationIO>
#include <tCalibration/CameraIO>
#include <tCalibration/EigenIO>
#include <tCalibration/ImuIO>
#include <tCamera/Camera>
#include <tCamera/OmnidirectionalCameraModel>
#include <tMotion/ImuNoise>

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

namespace {
constexpr char kTestLubaVioFile[]{"calib_parameters_sample.yaml"};
}

TEST(IoTest, EigenWrite)
{
    Matrix3d mat3d = Matrix3d::Random();
    const auto mat3d_yaml = io::toCVYamlNode(mat3d);
    LOG(INFO) << mat3d_yaml;

    Matrix4d mat4d = Matrix4d::Random();
    const auto mat4d_yaml = io::toCVYamlNode(mat4d);
    LOG(INFO) << mat4d_yaml;

    Vector3d vec3d = Vector3d::Random();
    const auto vec3d_yaml = io::toCVYamlNode(vec3d);
    LOG(INFO) << vec3d;

    EXPECT_TRUE(true);
}

TEST(IoTest, EigenRead)
{
    GTEST_SKIP();
    // TODO
}

TEST(IoTest, CameraWrite)
{
    GTEST_SKIP();
    // TOOD
}

TEST(IoTest, CameraRead)
{
    GTEST_SKIP();
    // TODO
}

TEST(IoTest, ImuWrite)
{
    GTEST_SKIP();

    constexpr imu::Type imu_type = imu::Type::Accelerator;
    ImuIntrinsics acc_intri{};
    const auto acc_intri_yaml = io::toYamlNode(acc_intri, imu_type);
    LOG(INFO) << acc_intri_yaml;

    ImuNoise acc_noise;
    const auto acc_noise_yaml = io::toYamlNode(acc_noise, imu_type);
    LOG(INFO) << acc_noise_yaml;

    EXPECT_TRUE(true);
}

TEST(IoTest, ImuRead)
{
    GTEST_SKIP();
    // TODO
}

TEST(IoTest, LubaVioWriteCompatiblity)
{
    GTEST_SKIP();

    io::CalibMetaData meta;

    Camera left{CameraIntrinsics::Type::Omnidirectional};
    Camera right{CameraIntrinsics::Type::Omnidirectional};
    ImuIntrinsics acc_intri;
    ImuIntrinsics gyr_intri;
    ImuNoise acc_noise;
    ImuNoise gyr_noise;

    const Matrix4d i_T_left = Matrix4d::Random();
    const Matrix4d i_T_right = Matrix4d::Random();

    const auto vio_yaml =
        io::toVioYamlString(left, right, i_T_left, i_T_right, meta);

    std::ofstream fout{kTestLubaVioFile};
    fout << vio_yaml;

    EXPECT_TRUE(true);
}

TEST(IoTest, LubaVioReadCompatibility)
{
    Camera left;
    Camera right;
    Eigen::Matrix4d leftCameraToImu;
    Eigen::Matrix4d rightCameraToImu;
    double focalLength;

    const bool success =
        io::loadFromVioYamlFile(kTestLubaVioFile, left, right, focalLength,
                                leftCameraToImu, rightCameraToImu);

    left.print();
    right.print();

    LOG(INFO) << "Left to imu: " << leftCameraToImu;
    LOG(INFO) << "Right to imu: " << rightCameraToImu;

    EXPECT_TRUE(success);
}
