#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tMotion/ImuIntrinsics>

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector3d;

template <typename T>
using Matrix34 = Eigen::Matrix<T, 3, 4>;

TEST(ImuIntrinsics, ctor)
{
    _ImuIntrinsics<double> intri;

    Matrix34<double> gt;
    gt.setZero();
    gt.block<3, 3>(0, 0).setIdentity();

    LOG(INFO) << intri;

    EXPECT_TRUE(gt.isApprox(intri.matrix()));
}

TEST(ImuIntrinsics, Misalginement)
{
    const Vector3d scale = Vector3d::Random();

    Matrix3d misalignment = Matrix3d::Random();
    misalignment.diagonal().setOnes();

    _ImuIntrinsics<double> intri;
    intri.scale() = scale;
    intri.setMisalignment(misalignment);

    EXPECT_TRUE(intri.scale().isApprox(scale))
        << "Scale is changed after setting misalignment.";

    {
        const Matrix3d gt = misalignment.triangularView<Eigen::StrictlyUpper>();

        EXPECT_TRUE(
            Matrix3d(
                intri.misalignment().triangularView<Eigen::StrictlyUpper>())
                .isApprox(gt));
        const auto _ =
            Matrix3d(
                intri.block<3, 3>(0, 0).triangularView<Eigen::StrictlyUpper>())
                .isApprox(Matrix3d(
                    misalignment.triangularView<Eigen::StrictlyUpper>()));
        EXPECT_TRUE(_);
    }
    {
        const Matrix3d gt = misalignment.triangularView<Eigen::StrictlyLower>();

        EXPECT_TRUE(
            Matrix3d(
                intri.misalignment().triangularView<Eigen::StrictlyLower>())
                .isApprox(gt));
        const auto _ =
            Matrix3d(
                intri.block<3, 3>(0, 0).triangularView<Eigen::StrictlyLower>())
                .isApprox(gt);
        EXPECT_TRUE(_);
    }
}

TEST(ImuIntrinsics, Scale)
{
    const Vector3d scale = Vector3d::Random();

    _ImuIntrinsics<double> intri;
    intri.scale() = scale;

    EXPECT_TRUE(intri.scale().isApprox(scale));
    {
        const auto _ = intri.block<3, 3>(0, 0).diagonal().isApprox(scale);
        EXPECT_TRUE(_);
    }
}

TEST(ImuIntrinsics, Bias)
{
    const Vector3d bias = Vector3d::Random();

    _ImuIntrinsics<double> intri;
    intri.bias() = bias;

    EXPECT_TRUE(intri.bias().isApprox(bias));
    EXPECT_TRUE(intri.col(3).isApprox(bias));
}
