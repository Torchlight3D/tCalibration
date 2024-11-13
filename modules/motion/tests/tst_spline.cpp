#include <format>

#include <gtest/gtest.h>

#include <tMotion/Spline/RdSpline>
#include <tMotion/Spline/SO3Spline>

#include "test_utils.h"

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Sophus::SO3d;

template <int _Dim, int _Order, int _Derivative>
void testEvaluate(const RdSpline<_Dim, _Order> &spline, int64_t time)
{
    using VectorD = typename RdSpline<_Dim, _Order>::VecD;
    using MatrixD = typename RdSpline<_Dim, _Order>::MatD;

    typename RdSpline<_Dim, _Order>::Jacobian J_spline;
    spline.template evaluate<_Derivative>(time, &J_spline);

    VectorD x0;
    x0.setZero();
    for (auto i{0}; i < 3 * _Order; i++) {
        MatrixD J_a;
        J_a.setZero();
        if (i >= J_spline.start_idx && i < J_spline.start_idx + _Order) {
            J_a.diagonal().setConstant(
                J_spline.d_val_d_knot[i - J_spline.start_idx]);
        }

        test_jacobian(
            std::format("d_val_d_knot{} time {}", i, time), J_a,
            [&](const VectorD &x) {
                auto _spline = spline;
                _spline.rKnot(i) += x;

                return _spline.template evaluate<_Derivative>(time);
            },
            x0);
    }
}

template <int _Dim, int _Order, int _Derivative>
void testTimeDeriv(const RdSpline<_Dim, _Order> &spline, int64_t time)
{
    using VectorD = typename RdSpline<_Dim, _Order>::VecD;

    const VectorD d_val_d_t = spline.template evaluate<_Derivative + 1>(time);

    Eigen::Matrix<double, 1, 1> x0;
    x0.setZero();
    test_jacobian(
        "d_val_d_t", d_val_d_t,
        [&](const Eigen::Matrix<double, 1, 1> &x) {
            int64_t inc = x[0] * 1e9;
            return spline.template evaluate<_Derivative>(time + inc);
        },
        x0);
}

template <int _Order>
void testEvaluateSo3(const So3Spline<_Order> &spline, int64_t time)
{
    using VectorD = typename So3Spline<_Order>::Vec3;
    using MatrixD = typename So3Spline<_Order>::Mat3;
    using SO3 = typename So3Spline<_Order>::SO3;

    typename So3Spline<_Order>::Jacobian J_spline;
    const SO3 res = spline.evaluate(time, &J_spline);

    VectorD x0;
    x0.setZero();
    for (auto i{0}; i < 3 * _Order; i++) {
        MatrixD J_a;
        J_a.setZero();
        if (i >= J_spline.start_idx && i < J_spline.start_idx + _Order) {
            J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
        }

        test_jacobian(
            std::format("d_val_d_knot{} time {}", i, time), J_a,
            [&](const VectorD &x) {
                auto _spline = spline;
                _spline.rKnot(i) = SO3::exp(x) * spline.knot(i);

                const SO3 _res = _spline.evaluate(time);
                return (_res * res.inverse()).log();
            },
            x0);
    }
}

template <int _Order>
void testVelSo3(const So3Spline<_Order> &spline, int64_t time)
{
    using VectorD = typename So3Spline<_Order>::Vec3;
    using SO3 = typename So3Spline<_Order>::SO3;

    const SO3 res = spline.evaluate(time);
    const VectorD d_res_d_t = spline.velocityBody(time);

    Eigen::Matrix<double, 1, 1> x0;
    x0.setZero();
    test_jacobian(
        "d_val_d_t", d_res_d_t,
        [&](const Eigen::Matrix<double, 1, 1> &x) {
            int64_t inc = x[0] * 1e9;
            return (res.inverse() * spline.evaluate(time + inc)).log();
        },
        x0);
}

template <int _Order>
void testAccelSo3(const So3Spline<_Order> &spline, int64_t time)
{
    using VectorD = typename So3Spline<5>::Vec3;

    VectorD vel1;
    const VectorD d_res_d_t = spline.accelerationBody(time, &vel1);

    const VectorD vel2 = spline.velocityBody(time);
    EXPECT_TRUE(vel1.isApprox(vel2));

    Eigen::Matrix<double, 1, 1> x0;
    x0.setZero();
    test_jacobian(
        "d_val_d_t", d_res_d_t,
        [&](const Eigen::Matrix<double, 1, 1> &x) {
            int64_t inc = x[0] * 1e9;
            return spline.velocityBody(time + inc);
        },
        x0);
}

template <int _Order>
void testJerkSo3(const So3Spline<_Order> &spline, int64_t time)
{
    using VectorD = typename So3Spline<5>::Vec3;

    VectorD vel1;
    VectorD accel1;
    VectorD d_res_d_t = spline.jerkBody(time, &vel1, &accel1);

    VectorD vel2 = spline.velocityBody(time);
    EXPECT_TRUE(vel1.isApprox(vel2));

    VectorD accel2 = spline.accelerationBody(time);
    EXPECT_TRUE(accel1.isApprox(accel2));

    Eigen::Matrix<double, 1, 1> x0;
    x0.setZero();
    test_jacobian(
        "d_val_d_t", d_res_d_t,
        [&](const Eigen::Matrix<double, 1, 1> &x) {
            int64_t inc = x[0] * 1e9;
            return spline.accelerationBody(time + inc);
        },
        x0);
}

template <int _Order>
void testEvaluateSo3Vel(const So3Spline<_Order> &spline, int64_t time)
{
    using VectorD = typename So3Spline<_Order>::Vec3;
    using MatrixD = typename So3Spline<_Order>::Mat3;
    using SO3 = typename So3Spline<_Order>::SO3;

    typename So3Spline<_Order>::Jacobian J_spline;
    const VectorD res = spline.velocityBody(time, &J_spline);
    const VectorD res_ref = spline.velocityBody(time);
    ASSERT_TRUE(res_ref.isApprox(res)) << "res and res_ref are not the same";

    VectorD x0;
    x0.setZero();
    for (auto i{0}; i < 3 * _Order; i++) {
        MatrixD J_a;
        J_a.setZero();
        if (i >= J_spline.start_idx && i < J_spline.start_idx + _Order) {
            J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
        }

        test_jacobian(
            std::format("d_vel_d_knot{} time {}", i, time), J_a,
            [&](const VectorD &x) {
                auto _spline = spline;
                _spline.rKnot(i) = SO3::exp(x) * spline.knot(i);

                return _spline.velocityBody(time);
            },
            x0);
    }
}

template <int _Order>
void testEvaluateSo3Accel(const So3Spline<_Order> &spline, int64_t time)
{
    using VectorD = typename So3Spline<_Order>::Vec3;
    using MatrixD = typename So3Spline<_Order>::Mat3;
    using SO3 = typename So3Spline<_Order>::SO3;

    VectorD vel;
    typename So3Spline<_Order>::Jacobian J_vel;
    typename So3Spline<_Order>::Jacobian J_accel;
    const VectorD res = spline.accelerationBody(time, &J_accel, &vel, &J_vel);

    VectorD vel_ref;
    const VectorD res_ref = spline.accelerationBody(time, &vel_ref);

    ASSERT_TRUE(vel_ref.isApprox(vel)) << "vel and vel_ref are not the same";
    ASSERT_TRUE(res_ref.isApprox(res)) << "res and res_ref are not the same";

    VectorD x0;
    x0.setZero();
    for (auto i{0}; i < 3 * _Order; i++) {
        MatrixD J_a;
        J_a.setZero();
        if (i >= J_vel.start_idx && i < J_vel.start_idx + _Order) {
            J_a = J_vel.d_val_d_knot[i - J_vel.start_idx];
        }

        test_jacobian(
            std::format("d_vel_d_knot{} time {}", i, time), J_a,
            [&](const VectorD &x) {
                auto _spline = spline;
                _spline.rKnot(i) = SO3::exp(x) * spline.knot(i);

                return _spline.velocityBody(time);
            },
            x0);
    }

    // Test acceleration Jacobian
    for (auto i{0}; i < 3 * _Order; i++) {
        MatrixD J_a;
        J_a.setZero();
        if (i >= J_accel.start_idx && i < J_accel.start_idx + _Order) {
            J_a = J_accel.d_val_d_knot[i - J_accel.start_idx];
        }

        test_jacobian(
            std::format("d_accel_d_knot{} time {}", i, time), J_a,
            [&](const VectorD &x) {
                auto _spline = spline;
                _spline.rKnot(i) = SO3::exp(x) * spline.knot(i);

                return _spline.accelerationBody(time);
            },
            x0);
    }
}

TEST(RdSpline, UBSplineEvaluateKnots4)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 4;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 0>(spline, time);
    }
}

TEST(RdSpline, UBSplineEvaluateKnots5)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 5;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 0>(spline, time);
    }
}

TEST(RdSpline, UBSplineEvaluateKnots6)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 6;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 0>(spline, time);
    }
}

TEST(RdSpline, UBSplineVelocityKnots4)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 4;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 1>(spline, time);
    }
}

TEST(RdSpline, UBSplineVelocityKnots5)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 5;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 1>(spline, time);
    }
}

TEST(RdSpline, UBSplineVelocityKnots6)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 6;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 1>(spline, time);
    }
}

TEST(RdSpline, UBSplineAccelKnots4)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 4;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 2>(spline, time);
    }
}

TEST(RdSpline, UBSplineAccelKnots5)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 5;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 2>(spline, time);
    }
}

TEST(RdSpline, UBSplineAccelKnots6)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 6;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluate<kDim, kOrder, 2>(spline, time);
    }
}

TEST(RdSpline, UBSplineEvaluateTimeDeriv4)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 4;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testTimeDeriv<kDim, kOrder, 0>(spline, time);
    }
}

TEST(RdSpline, UBSplineEvaluateTimeDeriv5)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 5;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testTimeDeriv<kDim, kOrder, 0>(spline, time);
    }
}

TEST(RdSpline, UBSplineEvaluateTimeDeriv6)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 6;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testTimeDeriv<kDim, kOrder, 0>(spline, time);
    }
}

TEST(RdSpline, UBSplineVelocityTimeDeriv4)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 4;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testTimeDeriv<kDim, kOrder, 1>(spline, time);
    }
}

TEST(RdSpline, UBSplineVelocityTimeDeriv5)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 5;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testTimeDeriv<kDim, kOrder, 1>(spline, time);
    }
}

TEST(RdSpline, UBSplineVelocityTimeDeriv6)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 6;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testTimeDeriv<kDim, kOrder, 1>(spline, time);
    }
}

// TODO: Finish this
TEST(RdSpline, UBSplineBounds)
{
    constexpr int kDim = 3;
    constexpr int kOrder = 5;

    RdSpline<kDim, kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    // std::cerr << "spline.maxTimeNs() " << spline.maxTimeNs() << std::endl;

    spline.evaluate(spline.endTime());
    // std::cerr << "res1\n" << res1.matrix() << std::endl;
    spline.evaluate(spline.startTime());
    // std::cerr << "res2\n" << res2.matrix() << std::endl;

    // Vector3d res3 = spline.evaluate(spline.maxTimeNs() + 1);
    // std::cerr << "res3\n" << res1.matrix() << std::endl;
    // Vector3d res4 = spline.evaluate(spline.minTimeNs() - 1);
    // std::cerr << "res4\n" << res2.matrix() << std::endl;
}

TEST(So3Spline, SO3CUBSplineEvaluateKnots4)
{
    constexpr int kOrder = 4;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineEvaluateKnots5)
{
    constexpr int kOrder = 5;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineEvaluateKnots6)
{
    constexpr int kOrder = 6;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineVelocity4)
{
    constexpr int kOrder = 4;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testVelSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineVelocity5)
{
    constexpr int kOrder = 5;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testVelSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineVelocity6)
{
    constexpr int kOrder = 6;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testVelSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineAcceleration4)
{
    constexpr int kOrder = 4;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testAccelSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineAcceleration5)
{
    constexpr int kOrder = 5;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testAccelSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineAcceleration6)
{
    constexpr int kOrder = 6;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testAccelSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineJerk5)
{
    constexpr int kOrder = 5;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testJerkSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineJerk6)
{
    constexpr int kOrder = 6;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 1e8; time < spline.endTime() - 1e8; time += 1e8) {
        testJerkSo3(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineVelocityKnots4)
{
    constexpr int kOrder = 4;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3Vel(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineVelocityKnots5)
{
    constexpr int kOrder = 5;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3Vel(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineVelocityKnots6)
{
    constexpr int kOrder = 6;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3Vel(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineAccelerationKnots4)
{
    constexpr int kOrder = 4;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3Accel(spline, time);
    }
}

TEST(So3Spline, SO3CUBSplineAccelerationKnots5)
{
    constexpr int kOrder = 5;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3Accel(spline, time);
    }
}

TEST(SplineTest, SO3CUBSplineAccelerationKnots6)
{
    constexpr int kOrder = 6;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        testEvaluateSo3Accel(spline, time);
    }
}

// TODO: Finish this
TEST(So3Spline, SO3CUBSplineBounds)
{
    constexpr int kOrder = 5;

    So3Spline<kOrder> spline(int64_t(2e9));
    spline.setRandom(3 * kOrder);

    // std::cerr << "spline.maxTimeNs() " << spline.maxTimeNs() << std::endl;

    spline.evaluate(spline.endTime());
    // std::cerr << "res1\n" << res1.matrix() << std::endl;
    spline.evaluate(spline.startTime());
    // std::cerr << "res2\n" << res2.matrix() << std::endl;

    // SO3d res3 = spline.evaluate(spline.maxTimeNs() + 1);
    // std::cerr << "res3\n" << res1.matrix() << std::endl;
    // SO3d res4 = spline.evaluate(spline.minTimeNs() - 1);
    // std::cerr << "res4\n" << res2.matrix() << std::endl;
}

TEST(SplineTest, CrossProductTest)
{
    Matrix3d J_1;
    Matrix3d J_2;
    Vector3d v1;
    Vector3d v2;
    J_1.setRandom();
    J_2.setRandom();
    v1.setRandom();
    v2.setRandom();

    {
        const Matrix3d J_cross =
            SO3d::hat(J_1 * v1) * J_2 - SO3d::hat(J_2 * v2) * J_1;
        test_jacobian(
            "cross_prod_test1", J_cross,
            [&](const Eigen::Vector3d &x) {
                return (J_1 * (v1 + x)).cross(J_2 * (v2 + x));
            },
            Vector3d::Zero());
    }

    {
        const Matrix3d J_cross = -SO3d::hat(J_2 * v2) * J_1;
        test_jacobian(
            "cross_prod_test2", J_cross,
            [&](const Eigen::Vector3d &x) {
                return (J_1 * (v1 + x)).cross(J_2 * v2);
            },
            Vector3d::Zero());
    }

    {
        const Matrix3d J_cross = SO3d::hat(J_1 * v1) * J_2;
        test_jacobian(
            "cross_prod_test2", J_cross,
            [&](const Eigen::Vector3d &x) {
                return (J_1 * v1).cross(J_2 * (v2 + x));
            },
            Vector3d::Zero());
    }
}
