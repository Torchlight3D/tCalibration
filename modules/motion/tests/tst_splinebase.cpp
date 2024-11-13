#include <gtest/gtest.h>

#include <tMotion/Spline/RdSpline>
#include <tMotion/Spline/SO3Spline>

#include "test_utils.h"

using namespace tl;

using Eigen::Vector3d;
using Sophus::SE3d;
using Sophus::Sim3d;
using Sophus::SO3d;
using Sophus::Vector6d;
using Sophus::Vector7d;

template <int _Order>
void testSO3Spline()
{
    constexpr int64_t kDt = 2e9;

    So3Spline<_Order> spline(kDt);
    spline.setRandom(3 * _Order);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        const SO3d pos1 = spline.evaluate(time);
        const Vector3d vel1 = spline.velocityBody(time);
        const Vector3d accel1 = spline.accelerationBody(time);
        const Vector3d jerk1 = spline.jerkBody(time);

        SO3d pos2;
        Vector3d vel2, accel2, jerk2;
        {
            const double pow_inv_dt = 1e9 / kDt;
            const auto [s, u] = internal::normalizeTime(time, 0, kDt);

            std::vector<const double *> vec;
            for (auto i{0}; i < _Order; i++) {
                vec.emplace_back(spline.knots()[s + i].data());
            }

            SplineBase<_Order>::template evaluate_lie<double, Sophus::SO3>(
                &vec[0], u, pow_inv_dt, &pos2);
            SplineBase<_Order>::template evaluate_lie<double, Sophus::SO3>(
                &vec[0], u, pow_inv_dt, nullptr, &vel2);
            SplineBase<_Order>::template evaluate_lie<double, Sophus::SO3>(
                &vec[0], u, pow_inv_dt, nullptr, nullptr, &accel2);
            SplineBase<_Order>::template evaluate_lie<double, Sophus::SO3>(
                &vec[0], u, pow_inv_dt, nullptr, nullptr, nullptr, &jerk2);
        }

        EXPECT_TRUE(pos1.matrix().isApprox(pos2.matrix()));
        EXPECT_TRUE(vel1.isApprox(vel2));
        EXPECT_TRUE(accel1.isApprox(accel2));
        EXPECT_TRUE(jerk1.isApprox(jerk2));
    }
}

template <int _Order>
void testRdSpline()
{
    constexpr int kDim = 3;
    constexpr int64_t kDt = 2e9;

    RdSpline<kDim, _Order> spline(kDt);
    spline.setRandom(3 * _Order);

    for (int64_t time = 0; time < spline.endTime(); time += 1e8) {
        const Vector3d pos1 = spline.template evaluate<0>(time);
        const Vector3d vel1 = spline.template evaluate<1>(time);
        const Vector3d accel1 = spline.template evaluate<2>(time);

        Vector3d pos2, vel2, accel2;
        {
            const double pow_inv_dt = 1e9 / kDt;
            const auto [s, u] = internal::normalizeTime(time, 0, kDt);

            std::vector<const double *> vec;
            for (auto i{0}; i < _Order; i++) {
                vec.emplace_back(spline.knots()[s + i].data());
            }

            SplineBase<_Order>::template evaluate<double, kDim, 0>(
                &vec[0], u, pow_inv_dt, &pos2);
            SplineBase<_Order>::template evaluate<double, kDim, 1>(
                &vec[0], u, pow_inv_dt, &vel2);
            SplineBase<_Order>::template evaluate<double, kDim, 2>(
                &vec[0], u, pow_inv_dt, &accel2);
        }

        EXPECT_TRUE(pos1.isApprox(pos2));
        EXPECT_TRUE(vel1.isApprox(vel2));
        EXPECT_TRUE(accel1.isApprox(accel2));
    }
}

template <int _Order>
void testSplineSE3Velocity()
{
    constexpr int64_t kDt = 2e9;

    std::vector<SE3d> knots;
    for (auto i{0}; i < 3 * _Order; i++) {
        knots.emplace_back(SE3d::exp(Vector6d::Random()));
    }

    for (auto i{0}; i < 2 * _Order; i++) {
        for (double u = 0.01; u < 0.99; u += 0.01) {
            Vector6d vel;
            SE3d pose;
            {
                const double pow_inv_dt = 1e9 / kDt;

                std::vector<const double *> vec;
                for (auto j{0}; j < _Order; j++) {
                    vec.emplace_back(knots[i + j].data());
                }

                SplineBase<_Order>::template evaluate_lie<double, Sophus::SE3>(
                    &vec[0], u, pow_inv_dt, &pose, &vel);

                Eigen::Matrix<double, 1, 1> x0;
                x0.setZero();
                test_jacobian(
                    "se3_vel", vel,
                    [&](const Eigen::Matrix<double, 1, 1> &x) {
                        const double inc = x[0] / (kDt * 1e-9);
                        SE3d pose_new;
                        SplineBase<_Order>::template evaluate_lie<double,
                                                                  Sophus::SE3>(
                            &vec[0], u + inc, pow_inv_dt, &pose_new);

                        return (pose.inverse() * pose_new).log();
                    },
                    x0);
            }
        }
    }
}

template <int _Order>
void testSplineSE3Acceleration()
{
    constexpr int64_t kDt = 2e9;

    std::vector<SE3d> knots;
    for (auto i{0}; i < 3 * _Order; ++i) {
        knots.emplace_back(SE3d::exp(Vector6d::Random()));
    }

    for (auto i{0}; i < 2 * _Order; ++i) {
        for (double u = 0.01; u < 0.99; u += 0.01) {
            Vector6d accel;
            {
                const double pow_inv_dt = 1e9 / kDt;

                std::vector<const double *> vec;
                for (auto j{0}; j < _Order; j++) {
                    vec.emplace_back(knots[i + j].data());
                }

                SplineBase<_Order>::template evaluate_lie<double, Sophus::SE3>(
                    &vec[0], u, pow_inv_dt, nullptr, nullptr, &accel);

                Eigen::Matrix<double, 1, 1> x0;
                x0.setZero();
                test_jacobian(
                    "se3_accel", accel,
                    [&](const Eigen::Matrix<double, 1, 1> &x) {
                        const double inc = x[0] / (kDt * 1e-9);
                        Vector6d vel;
                        SplineBase<_Order>::template evaluate_lie<double,
                                                                  Sophus::SE3>(
                            &vec[0], u + inc, pow_inv_dt, nullptr, &vel);

                        return vel;
                    },
                    x0);
            }
        }
    }
}

template <int _Order>
void testSplineSE3Jerk()
{
    constexpr int64_t kDt = 2e9;

    std::vector<SE3d> knots;
    for (auto i{0}; i < 3 * _Order; i++) {
        knots.emplace_back(SE3d::exp(Vector6d::Random()));
    }

    for (auto i{0}; i < 2 * _Order; i++) {
        for (double u = 0.01; u < 0.99; u += 0.01) {
            Vector6d jerk;
            {
                const double pow_inv_dt = 1e9 / kDt;

                std::vector<const double *> vec;
                for (int j = 0; j < _Order; j++) {
                    vec.emplace_back(knots[i + j].data());
                }

                SplineBase<_Order>::template evaluate_lie<double, Sophus::SE3>(
                    &vec[0], u, pow_inv_dt, nullptr, nullptr, nullptr, &jerk);

                Eigen::Matrix<double, 1, 1> x0;
                x0.setZero();
                test_jacobian(
                    "se3_jerk", jerk,
                    [&](const Eigen::Matrix<double, 1, 1> &x) {
                        const double inc = x[0] / (kDt * 1e-9);
                        Vector6d accel;
                        SplineBase<_Order>::template evaluate_lie<double,
                                                                  Sophus::SE3>(
                            &vec[0], u + inc, pow_inv_dt, nullptr, nullptr,
                            &accel);

                        return accel;
                    },
                    x0);
            }
        }
    }
}

template <int _Order>
void testSplineSim3Velocity()
{
    constexpr int64_t dt_ns = 2e9;

    std::vector<Sim3d> knots;
    for (auto i{0}; i < 3 * _Order; ++i) {
        knots.emplace_back(Sim3d::exp(Vector7d::Random()));
    }

    for (auto i{0}; i < 2 * _Order; ++i) {
        for (double u = 0.01; u < 0.99; u += 0.01) {
            Vector7d vel;
            Sim3d pose;
            {
                const double pow_inv_dt = 1e9 / dt_ns;

                std::vector<const double *> vec;
                for (auto j{0}; j < _Order; j++) {
                    vec.emplace_back(knots[i + j].data());
                }

                SplineBase<_Order>::template evaluate_lie<double, Sophus::Sim3>(
                    &vec[0], u, pow_inv_dt, &pose, &vel);

                Eigen::Matrix<double, 1, 1> x0;
                x0.setZero();
                test_jacobian(
                    "sim3_vel", vel,
                    [&](const Eigen::Matrix<double, 1, 1> &x) {
                        const double inc = x[0] / (dt_ns * 1e-9);
                        Sim3d pose_new;
                        SplineBase<_Order>::template evaluate_lie<double,
                                                                  Sophus::Sim3>(
                            &vec[0], u + inc, pow_inv_dt, &pose_new);

                        return (pose.inverse() * pose_new).log();
                    },
                    x0);
            }
        }
    }
}

template <int _Order>
void testSplineSim3Acceleration()
{
    constexpr int64_t kDt = 2e9;

    std::vector<Sim3d> knots;
    for (auto i{0}; i < 3 * _Order; i++) {
        knots.emplace_back(Sim3d::exp(Vector7d::Random()));
    }

    for (auto i{0}; i < 2 * _Order; i++) {
        for (double u = 0.01; u < 0.99; u += 0.01) {
            Vector7d accel;
            {
                const double pow_inv_dt = 1e9 / kDt;

                std::vector<const double *> vec;
                for (auto j{0}; j < _Order; j++) {
                    vec.emplace_back(knots[i + j].data());
                }

                SplineBase<_Order>::template evaluate_lie<double, Sophus::Sim3>(
                    &vec[0], u, pow_inv_dt, nullptr, nullptr, &accel);

                Eigen::Matrix<double, 1, 1> x0;
                x0.setZero();
                test_jacobian(
                    "sim3_accel", accel,
                    [&](const Eigen::Matrix<double, 1, 1> &x) {
                        const double inc = x[0] / (kDt * 1e-9);
                        Vector7d vel;
                        SplineBase<_Order>::template evaluate_lie<double,
                                                                  Sophus::Sim3>(
                            &vec[0], u + inc, pow_inv_dt, nullptr, &vel);

                        return vel;
                    },
                    x0);
            }
        }
    }
}

template <int _Order>
void testSplineSim3Jerk()
{
    constexpr int64_t kDt = 2e9;

    std::vector<Sim3d> knots;
    for (int i = 0; i < 3 * _Order; i++) {
        knots.emplace_back(Sim3d::exp(Vector7d::Random()));
    }

    for (auto i{0}; i < 2 * _Order; i++) {
        for (double u = 0.01; u < 0.99; u += 0.01) {
            Vector7d jerk;
            {
                const double pow_inv_dt = 1e9 / kDt;

                std::vector<const double *> vec;
                for (int j = 0; j < _Order; j++) {
                    vec.emplace_back(knots[i + j].data());
                }

                SplineBase<_Order>::template evaluate_lie<double, Sophus::Sim3>(
                    &vec[0], u, pow_inv_dt, nullptr, nullptr, nullptr, &jerk);

                Eigen::Matrix<double, 1, 1> x0;
                x0.setZero();
                test_jacobian(
                    "sim3_jerk", jerk,
                    [&](const Eigen::Matrix<double, 1, 1> &x) {
                        const double inc = x[0] / (kDt * 1e-9);
                        Vector7d accel;
                        SplineBase<_Order>::template evaluate_lie<double,
                                                                  Sophus::Sim3>(
                            &vec[0], u + inc, pow_inv_dt, nullptr, nullptr,
                            &accel);

                        return accel;
                    },
                    x0);
            }
        }
    }
}

TEST(SO3Spline, Order4) { testSO3Spline<4>(); }

TEST(SO3Spline, Order5) { testSO3Spline<5>(); }

TEST(SO3Spline, Order6) { testSO3Spline<6>(); }

TEST(RdSpline, Order4) { testRdSpline<4>(); }

TEST(RdSpline, Order5) { testRdSpline<5>(); }

TEST(RdSpline, Order6) { testRdSpline<6>(); }

TEST(SplineSE3Velocity, Order4) { testSplineSE3Velocity<4>(); }

TEST(SplineSE3Velocity, Order5) { testSplineSE3Velocity<5>(); }

TEST(SplineSE3Velocity, Order6) { testSplineSE3Velocity<6>(); }

TEST(SplineSE3Acceleration, Order4) { testSplineSE3Acceleration<4>(); }

TEST(SplineSE3Acceleration, Order5) { testSplineSE3Acceleration<5>(); }

TEST(SplineSE3Acceleration, Order6) { testSplineSE3Acceleration<6>(); }

TEST(SplineSE3Jerk, Order4) { testSplineSE3Jerk<4>(); }

TEST(SplineSE3Jerk, Order5) { testSplineSE3Jerk<5>(); }

TEST(SplineSE3Jerk, Order6) { testSplineSE3Jerk<6>(); }

TEST(SplineSim3Velocity, Order4) { testSplineSim3Velocity<4>(); }

TEST(SplineSim3Velocity, Order5) { testSplineSim3Velocity<5>(); }

TEST(SplineSim3Velocity, Order6) { testSplineSim3Velocity<6>(); }

TEST(Sim3Acceleration, Order4) { testSplineSim3Acceleration<4>(); }

TEST(Sim3Acceleration, Order5) { testSplineSim3Acceleration<5>(); }

TEST(Sim3Acceleration, Order6) { testSplineSim3Acceleration<6>(); }

TEST(SplineSim3Jerk, Order4) { testSplineSim3Jerk<4>(); }

TEST(SplineSim3Jerk, Order5) { testSplineSim3Jerk<5>(); }

TEST(SplineSim3Jerk, Order6) { testSplineSim3Jerk<6>(); }
