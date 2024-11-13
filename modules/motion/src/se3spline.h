#pragma once

#include <iostream>

#include <sophus/se3.hpp>

#include "imuintrinsics.h"
#include "rdspline.h"
#include "so3spline.h"

namespace tl {

template <int _Order, std::floating_point _Scalar = double>
class Se3Spline
{
public:
    using Scalar = _Scalar;

    enum
    {
        Order = _Order,
        Degree = _Order - 1
    };

    using MatN = Eigen::Matrix<Scalar, Order, Order>;
    using VecN = Eigen::Vector<Scalar, Order>;
    using VecNp1 = Eigen::Vector<Scalar, Order + 1>;

    using Vec3 = Eigen::Vector3<Scalar>;
    using Vec6 = Eigen::Vector<Scalar, 6>;
    using Vec9 = Eigen::Vector<Scalar, 9>;
    using Vec12 = Eigen::Vector<Scalar, 12>;

    using Mat3 = Eigen::Matrix3<Scalar>;
    using Mat6 = Eigen::Matrix<Scalar, 6, 6>;

    using Mat36 = Eigen::Matrix<Scalar, 3, 6>;
    using Mat39 = Eigen::Matrix<Scalar, 3, 9>;
    using Mat312 = Eigen::Matrix<Scalar, 3, 12>;

    using SO3 = Sophus::SO3<Scalar>;
    using SE3 = Sophus::SE3<Scalar>;

    using PosJacobian = typename RdSpline<3, Order, Scalar>::Jacobian;
    using SO3Jacobian = typename So3Spline<Order, Scalar>::Jacobian;

    /// @brief Struct to store the accelerometer residual Jacobian with
    /// respect to knots
    struct AccelPosSO3Jacobian
    {
        size_t start_idx;
        std::array<Mat36, Order> d_val_d_knot;
    };

    /// @brief Struct to store the pose Jacobian with respect to knots
    struct PosePosSO3Jacobian
    {
        size_t start_idx;
        std::array<Mat6, Order> d_val_d_knot;
    };

    explicit Se3Spline(int64_t dt, int64_t t0 = 0)
        : pos_spline_(dt, t0), so3_spline_(dt, t0)
    {
    }

    void setRandom(int n, bool static_init = false)
    {
        so3_spline_.setRandom(n, static_init);
        pos_spline_.setRandom(n, static_init);
    }

    inline int64_t timeInterval() const { return pos_spline_.timeInterval(); }
    int64_t startTime() const { return pos_spline_.startTime(); }
    inline void setStartTime(int64_t t0)
    {
        pos_spline_.setStartTime(t0);
        so3_spline_.setStartTime(t0);
    }
    int64_t endTime() const { return pos_spline_.endTime(); }

    inline SE3 frontKnot() const
    {
        return {so3_spline_.frontKnot(), pos_spline_.frontKnot()};
    }
    inline SE3 lastKnot() const
    {
        return {so3_spline_.knots().back(), pos_spline_.knots().back()};
    }

    SE3 knot(size_t i) const { return {knotSO3(i), knotPos(i)}; }
    inline SO3 &rKnotSO3(size_t i) { return so3_spline_.rKnot(i); }
    inline const SO3 &knotSO3(size_t i) const { return so3_spline_.knot(i); }
    inline Vec3 &rKnotPos(size_t i) { return pos_spline_.rKnot(i); }
    inline const Vec3 &knotPos(size_t i) const { return pos_spline_.knot(i); }

    size_t numKnots() const { return pos_spline_.knots().size(); }

    void setKnot(const Sophus::SE3d &pose, int i)
    {
        so3_spline_.rKnot(i) = pose.so3();
        pos_spline_.rKnot(i) = pose.translation();
    }

    /// @brief Reset spline to have num_knots initialized at pose
    ///
    /// @param[in] pose SE(3) pose
    /// @param[in] num_knots number of knots to initialize
    void setKnots(const Sophus::SE3d &pose, int numKnots)
    {
        so3_spline_.resize(numKnots);
        pos_spline_.resize(numKnots);
        for (int i = 0; i < numKnots; i++) {
            so3_spline_.rKnot(i) = pose.so3();
            pos_spline_.rKnot(i) = pose.translation();
        }
    }

    /// @brief Reset spline to the knots from other spline
    ///
    /// @param[in] other spline to copy knots from
    void setKnots(const Se3Spline<Order, Scalar> &other)
    {
        BASALT_ASSERT(other.pos_spline_.knots().size() ==
                      other.pos_spline_.knots().size());

        const size_t numKnots = other.pos_spline_.knots().size();

        so3_spline_.resize(numKnots);
        pos_spline_.resize(numKnots);
        for (size_t i = 0; i < numKnots; i++) {
            so3_spline_.rKnot(i) = other.so3_spline_.knot(i);
            pos_spline_.rKnot(i) = other.pos_spline_.knot(i);
        }
    }

    inline void knotsPushBack(const SE3 &knot)
    {
        so3_spline_.pushBack(knot.so3());
        pos_spline_.pushBack(knot.translation());
    }
    inline void knotsPopBack()
    {
        so3_spline_.popBack();
        pos_spline_.popBack();
    }
    inline void knotsPopFront()
    {
        so3_spline_.popFront();
        pos_spline_.popFront();
    }

    /// @brief Apply increment to the knot
    ///
    /// The incremernt vector consists of translational and rotational parts \f$
    /// [\upsilon, \omega]^T \f$. Given the current pose of the knot \f$ R \in
    /// SO(3), p \in \mathbb{R}^3\f$ the updated pose is: \f{align}{ R' &=
    /// \exp(\omega) R
    /// \\ p' &= p + \upsilon
    /// \f}
    ///  The increment is consistent with \ref
    /// PoseState::applyInc.
    ///
    /// @param[in] i index of the knot
    /// @param[in] inc 6x1 increment vector
    void applyInc(int i, const Vec6 &inc)
    {
        pos_spline_.rKnot(i) += inc.template head<3>();
        so3_spline_.rKnot(i) =
            SO3::exp(inc.template tail<3>()) * so3_spline_.knot(i);
    }

    /// @brief Linear acceleration in the world frame.
    ///
    /// @param[in] time_ns time to evaluate linear acceleration in nanoseconds
    inline Vec3 transAccelWorld(int64_t time) const
    {
        return pos_spline_.acceleration(time);
    }

    /// @brief Linear velocity in the world frame.
    ///
    /// @param[in] time_ns time to evaluate linear velocity in nanoseconds
    inline Vec3 transVelWorld(int64_t time) const
    {
        return pos_spline_.velocity(time);
    }

    /// @brief Rotational velocity in the body frame.
    ///
    /// @param[in] time_ns time to evaluate rotational velocity in nanoseconds
    inline Vec3 rotVelBody(int64_t time) const
    {
        return so3_spline_.velocityBody(time);
    }

    /// @brief Evaluate pose.
    ///
    /// @param[in] time_ns time to evaluate pose in nanoseconds
    /// @return SE(3) pose at time_ns
    SE3 pose(int64_t time) const
    {
        SE3 res;
        res.so3() = so3_spline_.evaluate(time);
        res.translation() = pos_spline_.evaluate(time);

        return res;
    }

    /// @brief Evaluate pose and compute Jacobian.
    ///
    /// @param[in] time_ns time to evaluate pose in nanoseconds
    /// @param[out] J Jacobian of the pose with respect to knots
    /// @return SE(3) pose at time_ns
    Sophus::SE3d pose(int64_t time, PosePosSO3Jacobian *J) const
    {
        Sophus::SE3d res;
        SO3Jacobian Jr;
        PosJacobian Jp;
        res.so3() = so3_spline_.evaluate(time, &Jr);
        res.translation() = pos_spline_.evaluate(time, &Jp);

        if (J) {
            const Eigen::Matrix3d RT = res.so3().inverse().matrix();

            J->start_idx = Jr.start_idx;
            for (auto i{0}; i < Order; ++i) {
                J->d_val_d_knot[i].setZero();
                J->d_val_d_knot[i].template topLeftCorner<3, 3>() =
                    RT * Jp.d_val_d_knot[i];
                J->d_val_d_knot[i].template bottomRightCorner<3, 3>() =
                    RT * Jr.d_val_d_knot[i];
            }
        }

        return res;
    }

    /// @brief Evaluate pose and compute time Jacobian.
    ///
    /// @param[in] time_ns time to evaluate pose in nanoseconds
    /// @param[out] J Jacobian of the pose with time
    void d_pose_d_t(int64_t time, Vec6 &J) const
    {
        J.template head<3>() =
            so3_spline_.evaluate(time).inverse() * transVelWorld(time);
        J.template tail<3>() = rotVelBody(time);
    }

    /// @brief Evaluate gyroscope residual.
    ///
    /// @param[in] time_ns time of the measurement
    /// @param[in] measurement gyroscope measurement
    /// @param[in] gyro_bias_full gyroscope calibration
    /// @return gyroscope residual
    Vec3 gyroResidual(int64_t time, const Vec3 &measurement,
                      const CalibGyroBias<Scalar> &gyro_bias_full) const
    {
        return so3_spline_.velocityBody(time) -
               gyro_bias_full.getCalibrated(measurement);
    }

    /// @brief Evaluate gyroscope residual and compute Jacobians.
    ///
    /// @param[in] time_ns time of the measurement
    /// @param[in] measurement gyroscope measurement
    /// @param[in] gyro_bias_full gyroscope calibration
    /// @param[out] J_knots Jacobian with respect to SO(3) spline knots
    /// @param[out] J_bias Jacobian with respect to gyroscope calibration
    /// @return gyroscope residual
    Vec3 gyroResidual(int64_t time, const Vec3 &measurement,
                      const CalibGyroBias<Scalar> &gyro_bias_full,
                      SO3Jacobian *J_knots, Mat312 *J_bias = nullptr) const
    {
        if (J_bias) {
            J_bias->setZero();
            J_bias->template block<3, 3>(0, 0).diagonal().array() = 1.0;
            J_bias->template block<3, 3>(0, 3).diagonal().array() =
                -measurement[0];
            J_bias->template block<3, 3>(0, 6).diagonal().array() =
                -measurement[1];
            J_bias->template block<3, 3>(0, 9).diagonal().array() =
                -measurement[2];
        }

        return so3_spline_.velocityBody(time, J_knots) -
               gyro_bias_full.getCalibrated(measurement);
    }

    /// @brief Evaluate accelerometer residual.
    ///
    /// @param[in] time_ns time of the measurement
    /// @param[in] measurement accelerometer measurement
    /// @param[in] accel_bias_full accelerometer calibration
    /// @param[in] g gravity
    /// @return accelerometer residual
    Vec3 accelResidual(int64_t time, const Eigen::Vector3d &measurement,
                       const CalibAccelBias<Scalar> &accel_bias_full,
                       const Eigen::Vector3d &g) const
    {
        Sophus::SO3d R = so3_spline_.evaluate(time);
        Eigen::Vector3d accel_world = pos_spline_.acceleration(time);

        return R.inverse() * (accel_world + g) -
               accel_bias_full.getCalibrated(measurement);
    }

    /// @brief Evaluate accelerometer residual and Jacobians.
    ///
    /// @param[in] time_ns time of the measurement
    /// @param[in] measurement accelerometer measurement
    /// @param[in] accel_bias_full accelerometer calibration
    /// @param[in] g gravity
    /// @param[out] J_knots Jacobian with respect to spline knots
    /// @param[out] J_bias Jacobian with respect to accelerometer calibration
    /// @param[out] J_g Jacobian with respect to gravity
    /// @return accelerometer residual
    Vec3 accelResidual(int64_t time, const Vec3 &measurement,
                       const CalibAccelBias<Scalar> &accel_bias_full,
                       const Vec3 &g, AccelPosSO3Jacobian *J_knots,
                       Mat39 *J_bias = nullptr, Mat3 *J_g = nullptr) const
    {
        SO3Jacobian Jr;
        const Sophus::SO3d R = so3_spline_.evaluate(time, &Jr);

        PosJacobian Jp;
        const Eigen::Vector3d accel_world = pos_spline_.acceleration(time, &Jp);

        const Eigen::Matrix3d RT = R.inverse().matrix();
        const Eigen::Matrix3d tmp = RT * Sophus::SO3d::hat(accel_world + g);

        // BASALT_ASSERT_STREAM(Jr.start_idx == Jp.start_idx,
        //                      "Jr.start_idx " << Jr.start_idx << "
        //                      Jp.start_idx "
        //                                      << Jp.start_idx);

        // BASALT_ASSERT_STREAM(
        //     so3_spline_.getKnots().size() == pos_spline_.getKnots().size(),
        //     "so3_spline.getKnots().size() " << so3_spline_.getKnots().size()
        //                                     << " pos_spline.getKnots().size()
        //                                     "
        //                                     <<
        //                                     pos_spline_.getKnots().size());

        J_knots->start_idx = Jp.start_idx;
        for (int i = 0; i < Order; i++) {
            J_knots->d_val_d_knot[i].template topLeftCorner<3, 3>() =
                RT * Jp.d_val_d_knot[i];
            J_knots->d_val_d_knot[i].template bottomRightCorner<3, 3>() =
                tmp * Jr.d_val_d_knot[i];
        }

        if (J_bias) {
            J_bias->setZero();
            J_bias->template block<3, 3>(0, 0).diagonal().array() = 1.0;
            J_bias->template block<3, 3>(0, 3).diagonal().array() =
                -measurement[0];
            (*J_bias)(1, 6) = -measurement[1];
            (*J_bias)(2, 7) = -measurement[1];
            (*J_bias)(2, 8) = -measurement[2];
        }
        if (J_g) {
            (*J_g) = RT;
        }

        Vec3 res =
            RT * (accel_world + g) - accel_bias_full.getCalibrated(measurement);
        return res;
    }

    /// @brief Evaluate position residual.
    ///
    /// @param[in] time_ns time of the measurement
    /// @param[in] measured_position position measurement
    /// @param[out] Jp if not nullptr, Jacobian with respect to knos of the
    /// position spline
    /// @return position residual
    Sophus::Vector3d positionResidual(int64_t time, const Vec3 &position,
                                      PosJacobian *Jp = nullptr) const
    {
        return pos_spline_.evaluate(time, Jp) - position;
    }

    /// @brief Evaluate orientation residual.
    ///
    /// @param[in] time_ns time of the measurement
    /// @param[in] measured_orientation orientation measurement
    /// @param[out] Jr if not nullptr, Jacobian with respect to knos of the
    /// SO(3) spline
    /// @return orientation residual
    Sophus::Vector3d orientationResidual(int64_t time, const SO3 &orientation,
                                         SO3Jacobian *Jr = nullptr) const
    {
        Sophus::Vector3d res =
            (so3_spline_.evaluate(time, Jr) * orientation.inverse()).log();

        if (Jr) {
            Eigen::Matrix3d Jrot;
            Sophus::leftJacobianSO3(res, Jrot);

            for (int i = 0; i < Order; i++) {
                Jr->d_val_d_knot[i] = Jrot * Jr->d_val_d_knot[i];
            }
        }

        return res;
    }

    /// Debug
    friend std::ostream &operator<<(std::ostream &os,
                                    const Se3Spline<Order, Scalar> &spline);

private:
    RdSpline<3, Order, Scalar> pos_spline_;
    So3Spline<Order, Scalar> so3_spline_;
};

template <int _Order, typename _Scalar>
std::ostream &operator<<(std::ostream &os,
                         const Se3Spline<_Order, _Scalar> &spline)
{
    os << "SE3 Spline knots:"
          "\n";
    const auto &positions = spline.pos_spline_;
    const auto &orientations = spline.so3_spline_;
    for (size_t i{0}; i < positions.getKnots().size(); i++) {
        os << i << ": position:" << positions.getKnot(i).transpose()
           << " orientation: "
           << orientations.getKnot(i).unit_quaternion().coeffs().transpose()
           << "\n";
    }
}

} // namespace tl
