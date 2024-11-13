#include <format>

#include <gtest/gtest.h>

#include <tMotion/Spline/SE3Spline>

#include "test_utils.h"

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Sophus::SE3d;
using Sophus::SO3d;
using Sophus::Vector6d;

template <int _Order>
void testGyroRes(const Se3Spline<_Order> &spline, int64_t time)
{
    CalibGyroBias<double> bias;
    bias.setRandom();
    // bias << 0.01, -0.02, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    const Vector3d measurement = spline.rotVelBody(time);

    typename Se3Spline<_Order>::SO3Jacobian J_spline;
    Eigen::Matrix<double, 3, 12> J_bias;
    spline.gyroResidual(time, measurement, bias, &J_spline, &J_bias);

    for (size_t i = 0; i < spline.numKnots(); i++) {
        Matrix3d J_a;
        if (i >= J_spline.start_idx && i < J_spline.start_idx + _Order) {
            J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
        }
        else {
            J_a.setZero();
        }

        Vector3d x0;
        x0.setZero();
        test_jacobian(
            std::format("Spline order {} d_gyro_res_d_knot{} time {}", _Order,
                        i, time),
            J_a,
            [&](const Eigen::Vector3d &x) {
                auto _spline = spline;
                _spline.rKnotSO3(i) = SO3d::exp(x) * spline.knotSO3(i);

                return _spline.gyroResidual(time, measurement, bias);
            },
            x0);
    }

    {
        Eigen::Matrix<double, 12, 1> x0;
        x0.setZero();
        test_jacobian(
            std::format("Spline order {} d_gyro_res_d_bias", _Order), J_bias,
            [&](const Eigen::Matrix<double, 12, 1> &x) {
                auto _bias = bias;
                _bias += x;

                return spline.gyroResidual(time, measurement, _bias);
            },
            x0);
    }
}

template <int _Order>
void testAccelRes(const Se3Spline<_Order> &spline, int64_t time)
{
    CalibAccelBias<double> bias;
    bias.setRandom();
    // bias << -0.4, 0.5, -0.6, 0, 0, 0, 0, 0, 0;

    const Vector3d g(0, 0, -9.81);
    const Vector3d measurement = spline.transAccelWorld(time) + g;

    typename Se3Spline<_Order>::AccelPosSO3Jacobian J_spline;
    Eigen::Matrix<double, 3, 9> J_bias;
    Matrix3d J_g;
    spline.accelResidual(time, measurement, bias, g, &J_spline, &J_bias, &J_g);

    for (size_t i = 0; i < spline.numKnots(); i++) {
        typename Se3Spline<_Order>::Mat36 J_a;
        if (i >= J_spline.start_idx && i < J_spline.start_idx + _Order) {
            J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
        }
        else {
            J_a.setZero();
        }

        Vector6d x0;
        x0.setZero();
        test_jacobian(
            std::format("Spline order {} d_accel_res_d_knot{}", _Order, i), J_a,
            [&](const Sophus::Vector6d &x) {
                auto _spline = spline;
                _spline.applyInc(i, x);

                return _spline.accelResidual(time, measurement, bias, g);
            },
            x0);
    }

    {
        Eigen::Matrix<double, 9, 1> x0;
        x0.setZero();
        test_jacobian(
            std::format("Spline order {} d_accel_res_d_bias", _Order), J_bias,
            [&](const Eigen::Matrix<double, 9, 1> &x) {
                auto _bias = bias;
                _bias += x;

                return spline.accelResidual(time, measurement, _bias, g);
            },
            x0);
    }

    {
        Vector3d x0;
        x0.setZero();
        test_jacobian(
            std::format("Spline order {} d_accel_res_d_g", _Order), J_g,
            [&](const Eigen::Vector3d &x) {
                return spline.accelResidual(time, measurement, bias, g + x);
            },
            x0);
    }
}

template <int _Order>
void testOrientationRes(const Se3Spline<_Order> &spline, int64_t time)
{
    const SO3d measurement = spline.pose(time).so3();
    // Sophus::expd(Vector6d::Random() / 10);

    typename Se3Spline<_Order>::SO3Jacobian J_spline;
    spline.orientationResidual(time, measurement, &J_spline);

    for (size_t i = 0; i < spline.numKnots(); i++) {
        Matrix3d J_a;
        if (i >= J_spline.start_idx && i < J_spline.start_idx + _Order) {
            J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
        }
        else {
            J_a.setZero();
        }

        Vector3d x0;
        x0.setZero();
        test_jacobian(
            std::format("Spline order {} d_rot_res_d_knot{}", _Order, i), J_a,
            [&](const Eigen::Vector3d &x_rot) {
                Vector6d x;
                x.setZero();
                x.tail<3>() = x_rot;

                auto _spline = spline;
                _spline.applyInc(i, x);

                return _spline.orientationResidual(time, measurement);
            },
            x0);
    }
}

template <int _Order>
void testPositionRes(const Se3Spline<_Order> &spline, int64_t time)
{
    // * Sophus::expd(Vector6d::Random() / 10);
    const Vector3d measurement = spline.pose(time).translation();

    typename Se3Spline<_Order>::PosJacobian J_spline;
    spline.positionResidual(time, measurement, &J_spline);

    for (size_t i = 0; i < spline.numKnots(); i++) {
        Matrix3d J_a;
        J_a.setZero();
        if (i >= J_spline.start_idx && i < J_spline.start_idx + _Order) {
            J_a.diagonal().setConstant(
                J_spline.d_val_d_knot[i - J_spline.start_idx]);
        }

        Vector3d x0;
        x0.setZero();
        test_jacobian(
            std::format("Spline order {} d_pos_res_d_knot{}", _Order, i), J_a,
            [&](const Eigen::Vector3d &x_rot) {
                Vector6d x;
                x.setZero();
                x.head<3>() = x_rot;

                auto _spline = spline;
                _spline.applyInc(i, x);

                return _spline.positionResidual(time, measurement);
            },
            x0);
    }
}

template <int _Order>
void testPose(const Se3Spline<_Order> &spline, int64_t time)
{
    typename Se3Spline<_Order>::PosePosSO3Jacobian J_spline;
    const SE3d res = spline.pose(time, &J_spline);

    Vector6d x0;
    x0.setZero();
    for (size_t i = 0; i < spline.numKnots(); i++) {
        typename Se3Spline<_Order>::Mat6 J_a;
        if (i >= J_spline.start_idx && i < J_spline.start_idx + _Order) {
            J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
        }
        else {
            J_a.setZero();
        }

        test_jacobian(
            std::format("Spline order {} d_pose_d_knot{}", _Order, i), J_a,
            [&](const Sophus::Vector6d &x) {
                auto _spline = spline;
                _spline.applyInc(i, x);

                return Sophus::se3_logd(res.inverse() * _spline.pose(time));
            },
            x0);
    }

    {
        Eigen::Matrix<double, 1, 1> x0;
        x0.setZero();

        typename Se3Spline<_Order>::Vec6 J_a;
        spline.d_pose_d_t(time, J_a);

        // J.template head<3>() = res.so3().inverse() * s.transVelWorld(time);
        // J.template tail<3>() = s.rotVelBody(time);

        test_jacobian(
            "J_pose_time", J_a,
            [&](const Eigen::Matrix<double, 1, 1> &x) {
                auto _time = time;
                _time += x[0] * 1e9;

                return Sophus::se3_logd(res.inverse() * spline.pose(_time));
            },
            x0);
    }
}

TEST(SE3Spline, GyroResidualTest)
{
    constexpr int kOrder = 5;
    constexpr int kNumKnots = 3 * kOrder;

    Se3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(kNumKnots);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testGyroRes<kOrder>(spline, time);
    }
}

TEST(SE3Spline, AccelResidualTest)
{
    constexpr int kOrder = 5;
    constexpr int kNumKnots = 3 * kOrder;

    Se3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(kNumKnots);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testAccelRes<kOrder>(spline, time);
    }
}

TEST(SE3Spline, PositionResidualTest)
{
    constexpr int kOrder = 5;
    constexpr int kNumKnots = 3 * kOrder;

    Se3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(kNumKnots);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testPositionRes<kOrder>(spline, time);
    }
}

TEST(SE3Spline, OrientationResidualTest)
{
    constexpr int kOrder = 5;
    constexpr int kNumKnots = 3 * kOrder;

    Se3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(kNumKnots);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testOrientationRes<kOrder>(spline, time);
    }
}

TEST(SE3Spline, PoseTest)
{
    constexpr int kOrder = 5;
    constexpr int kNumKnots = 3 * kOrder;

    Se3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(kNumKnots);

    constexpr int64_t kOffset = 100;
    for (int64_t time = kOffset; time < spline.endTime() - kOffset;
         time += 1e8) {
        testPose<kOrder>(spline, time);
    }
}
