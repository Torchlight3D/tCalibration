#pragma once

#include <deque>

#include <sophus/so3.hpp>

#include "splinebase.h"
#include "utils.h"

namespace tl {

/// @brief Uniform cummulative B-spline for SO(3) of order N
///
/// For example, in the particular case scalar values and order N=5, for a time
/// \f$t \in [t_i, t_{i+1})\f$ the value of \f$p(t)\f$ depends only on 5 control
/// points at \f$[t_i, t_{i+1}, t_{i+2}, t_{i+3}, t_{i+4}]\f$. To
/// simplify calculations we transform time to uniform representation \f$s(t) =
/// (t - t_0)/\Delta t \f$, such that control points transform into \f$ s_i \in
/// [0,..,N] \f$. We define function \f$ u(t) = s(t)-s_i \f$ to be a time since
/// the start of the segment. Following the cummulative matrix representation of
/// De Boor - Cox formula, the value of the function can be evaluated as
/// follows: \f{align}{
///    R(u(t)) &= R_i
///    \prod_{j=1}^{4}{\exp(k_{j}\log{(R_{i+j-1}^{-1}R_{i+j})})},
///    \\ \begin{pmatrix} k_0 \\ k_1 \\ k_2 \\ k_3 \\ k_4 \end{pmatrix}^T &=
///    M_{c5} \begin{pmatrix} 1 \\ u \\ u^2 \\ u^3 \\ u^4
///    \end{pmatrix},
/// \f}
/// where \f$ R_{i} \in SO(3) \f$ are knots and \f$ M_{c5} \f$ is a cummulative
/// blending matrix computed using \ref computeBlendingMatrix \f{align}{
///    M_{c5} = \frac{1}{4!}
///    \begin{pmatrix} 24 & 0 & 0 & 0 & 0 \\ 23 & 4 & -6 & 4 & -1 \\ 12 & 16 & 0
///    & -8 & 3 \\ 1 & 4 & 6 & 4 & -3 \\ 0 & 0 & 0 & 0 & 1 \end{pmatrix}.
/// \f}
///
/// See [[arXiv:1911.08860]](https://arxiv.org/abs/1911.08860) for more details.
template <int _Order, std::floating_point _Scalar = double>
class So3Spline : public SplineBase<_Order, _Scalar>
{
    using Base = SplineBase<_Order, _Scalar>;

public:
    using Scalar = Base::Scalar;

    enum
    {
        Order = Base::Order,
        Degree = Base::Degree,
    };

    // TODO: Remove this
    static constexpr _Scalar NS_TO_S = 1e-9;
    static constexpr _Scalar S_TO_NS = 1e9;

    using MatN = Base::MatN;
    using VecN = Base::VecN;

    using Mat3 = Eigen::Matrix3<Scalar>;
    using Vec3 = Eigen::Vector3<Scalar>;

    using SO3 = Sophus::SO3<Scalar>;

    /// @brief Struct to store the Jacobian of the spline
    ///
    /// Since B-spline of order N has local support (only N knots infuence the
    /// value) the Jacobian is zero for all knots except maximum N for value and
    /// all derivatives.
    struct Jacobian
    {
        size_t start_idx;
        std::array<Mat3, Order> d_val_d_knot;
    };

    explicit So3Spline(int64_t dt, int64_t t0 = 0) : _t0(t0), _dt(dt)
    {
        pow_inv_dt_[0] = 1.0;
        pow_inv_dt_[1] = S_TO_NS / _dt;
        pow_inv_dt_[2] = pow_inv_dt_[1] * pow_inv_dt_[1];
        pow_inv_dt_[3] = pow_inv_dt_[2] * pow_inv_dt_[1];
    }

    int64_t timeInterval() const { return _dt; }
    int64_t startTime() const { return _t0; }
    inline void setStartTime(int64_t t0) { _t0 = t0; }
    int64_t endTime() const
    {
        return _t0 + (_knots.size() - Order + 1) * _dt - 1;
    }

    /// @brief Gererate random trajectory
    ///
    /// @param[in] n number of knots to generate
    /// @param[in] static_init if true the first N knots will be the same
    /// resulting in static initial condition
    void setRandom(int n, bool static_init = false)
    {
        if (static_init) {
            Vec3 rnd = Vec3::Random() * M_PI;

            for (auto i{0}; i < Order; ++i) {
                _knots.push_back(SO3::exp(rnd));
            }

            for (auto i{0}; i < n - Order; ++i) {
                _knots.push_back(_knots.back() *
                                 SO3::exp(Vec3::Random() * M_PI / 2));
            }
        }
        else {
            _knots.push_back(SO3::exp(Vec3::Random() * M_PI));
            for (auto i{1}; i < n; ++i) {
                _knots.push_back(_knots.back() *
                                 SO3::exp(Vec3::Random() * M_PI / 2));
            }
        }
    }

    inline const SO3& frontKnot() const { return _knots.front(); }
    inline SO3& rKnot(int i) { return _knots[i]; }
    inline const SO3& knot(int i) const { return _knots[i]; }
    inline const std::deque<SO3>& knots() const { return _knots; }

    inline void pushBack(const SO3& knot) { _knots.push_back(knot); }
    inline void popFront()
    {
        _t0 += _dt;
        _knots.pop_front();
    }
    inline void popBack() { _knots.pop_back(); }

    inline void resize(size_t n) { _knots.resize(n); }

    /// @brief Evaluate SO(3) B-spline
    ///
    /// @param[in] time_ns time for evaluating the value of the spline in
    /// nanoseconds
    /// @param[out] J if not nullptr, return the Jacobian of the value with
    /// respect to knots
    /// @return SO(3) value of the spline
    SO3 evaluate(int64_t time, Jacobian* J = nullptr) const
    {
        const auto [s, u] = internal::normalizeTime(time, _t0, _dt);

        VecN p = Base::baseCoeffsWithTime<0>(u);
        const VecN coeff = this->kCumulativeBlendingMatrix * p;

        Mat3 J_helper;
        if (J) {
            J->start_idx = s;
            J_helper.setIdentity();
        }

        SO3 res = _knots[s];
        for (auto i{0}; i < Degree; i++) {
            const SO3& p0 = _knots[s + i];
            const SO3& p1 = _knots[s + i + 1];

            SO3 r01 = p0.inverse() * p1;
            Vec3 delta = r01.log();
            Vec3 kdelta = delta * coeff[i + 1];

            if (J) {
                Mat3 Jl_inv_delta;
                Mat3 Jl_k_delta;
                Sophus::leftJacobianInvSO3(delta, Jl_inv_delta);
                Sophus::leftJacobianSO3(kdelta, Jl_k_delta);

                J->d_val_d_knot[i] = J_helper;
                J_helper = coeff[i + 1] * res.matrix() * Jl_k_delta *
                           Jl_inv_delta * p0.inverse().matrix();
                J->d_val_d_knot[i] -= J_helper;
            }
            res *= SO3::exp(kdelta);
        }

        if (J) {
            J->d_val_d_knot[Degree] = J_helper;
        }

        return res;
    }

    /// @brief Evaluate rotational velocity (first time derivative) of SO(3)
    /// B-spline in the body frame

    /// First, let's note that for scalars \f$ k, \Delta k \f$ the following
    /// holds: \f$ \exp((k+\Delta k)\phi) = \exp(k\phi)\exp(\Delta k\phi), \phi
    /// \in \mathbb{R}^3\f$. This is due to the fact that rotations around the
    /// same axis are commutative.
    ///
    /// Let's take SO(3) B-spline with N=3 as an example. The evolution in time
    /// of rotation from the body frame to the world frame is described with \f[
    ///  R_{wb}(t) = R(t) = R_i \exp(k_1(t) \log(R_{i}^{-1}R_{i+1})) \exp(k_2(t)
    ///  \log(R_{i+1}^{-1}R_{i+2})), \f] where \f$ k_1, k_2 \f$ are spline
    ///  coefficients (see detailed description of \ref So3Spline). Since
    ///  expressions under logmap do not depend on time we can rename them to
    ///  constants.
    /// \f[ R(t) = R_i \exp(k_1(t) ~ d_1) \exp(k_2(t) ~ d_2). \f]
    ///
    /// With linear approximation of the spline coefficient evolution over time
    /// \f$ k_1(t_0 + \Delta t) = k_1(t_0) + k_1'(t_0)\Delta t \f$ we can write
    /// \f{align}
    ///  R(t_0 + \Delta t) &= R_i \exp( (k_1(t_0) + k_1'(t_0) \Delta t) ~ d_1)
    ///  \exp((k_2(t_0) + k_2'(t_0) \Delta t) ~ d_2)
    ///  \\ &= R_i \exp(k_1(t_0) ~ d_1) \exp(k_1'(t_0)~ d_1 \Delta t )
    ///  \exp(k_2(t_0) ~ d_2) \exp(k_2'(t_0) ~ d_2 \Delta t )
    ///  \\ &= R_i \exp(k_1(t_0) ~ d_1)
    ///  \exp(k_2(t_0) ~ d_2) \exp(R_{a}^T k_1'(t_0)~ d_1 \Delta t )
    ///  \exp(k_2'(t_0) ~ d_2 \Delta t )
    ///  \\ &= R_i \exp(k_1(t_0) ~ d_1)
    ///  \exp(k_2(t_0) ~ d_2) \exp((R_{a}^T k_1'(t_0)~ d_1 +
    ///  k_2'(t_0) ~ d_2) \Delta t )
    ///  \\ &= R(t_0) \exp((R_{a}^T k_1'(t_0)~ d_1 +
    ///  k_2'(t_0) ~ d_2) \Delta t )
    ///  \\ &= R(t_0) \exp( \omega \Delta t ),
    /// \f} where \f$ \Delta t \f$ is small, \f$ R_{a} \in SO(3) = \exp(k_2(t_0)
    /// ~ d_2) \f$ and \f$ \omega \f$ is the rotational velocity in the body
    /// frame. More explicitly we have the formula for rotational velocity in
    /// the body frame \f[ \omega = R_{a}^T k_1'(t_0)~ d_1 +  k_2'(t_0) ~ d_2.
    /// \f] Derivatives of spline coefficients can be computed with \ref
    /// baseCoeffsWithTime similar to \ref RdSpline (detailed description). With
    /// the recursive formula computations generalize to different orders of
    /// spline N.
    ///
    /// @param[in] time_ns time for evaluating velocity of the spline in
    /// nanoseconds
    /// @return rotational velocity (3x1 vector)
    Vec3 velocityBody(int64_t time) const
    {
        const auto [s, u] = internal::normalizeTime(time, _t0, _dt);

        VecN p = Base::baseCoeffsWithTime<0>(u);
        const VecN coeff = this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<1>(u);
        VecN dcoeff = pow_inv_dt_[1] * this->kCumulativeBlendingMatrix * p;

        Vec3 rot_vel;
        rot_vel.setZero();
        for (auto i{0}; i < Degree; ++i) {
            const SO3& p0 = _knots[s + i];
            const SO3& p1 = _knots[s + i + 1];

            SO3 r01 = p0.inverse() * p1;
            Vec3 delta = r01.log();

            rot_vel = SO3::exp(-delta * coeff[i + 1]) * rot_vel;
            rot_vel += delta * dcoeff[i + 1];
        }

        return rot_vel;
    }

    /// @brief Evaluate rotational velocity (first time derivative) of SO(3)
    /// B-spline in the body frame
    ///
    /// @param[in] time_ns time for evaluating velocity of the spline in
    /// nanoseconds
    /// @param[out] J if not nullptr, return the Jacobian of the rotational
    /// velocity in body frame with respect to knots
    /// @return rotational velocity (3x1 vector)
    Vec3 velocityBody(int64_t time, Jacobian* J) const
    {
        const auto [s, u] = internal::normalizeTime(time, _t0, _dt);

        VecN p = Base::baseCoeffsWithTime<0>(u);
        const VecN coeff = this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<1>(u);
        const VecN dcoeff =
            pow_inv_dt_[1] * this->kCumulativeBlendingMatrix * p;

        Vec3 delta_vec[Degree];
        Mat3 R_tmp[Degree];
        SO3 accum;
        SO3 exp_k_delta[Degree];
        Mat3 Jr_delta_inv[Degree];
        Mat3 Jr_kdelta[Degree];
        for (int i = Degree - 1; i >= 0; i--) {
            const SO3& p0 = _knots[s + i];
            const SO3& p1 = _knots[s + i + 1];

            SO3 r01 = p0.inverse() * p1;
            delta_vec[i] = r01.log();

            Sophus::rightJacobianInvSO3(delta_vec[i], Jr_delta_inv[i]);
            Jr_delta_inv[i] *= p1.inverse().matrix();

            Vec3 k_delta = coeff[i + 1] * delta_vec[i];
            Sophus::rightJacobianSO3(-k_delta, Jr_kdelta[i]);

            R_tmp[i] = accum.matrix();
            exp_k_delta[i] = Sophus::SO3d::exp(-k_delta);
            accum *= exp_k_delta[i];
        }

        Mat3 d_vel_d_delta[Degree];
        d_vel_d_delta[0] = dcoeff[1] * R_tmp[0] * Jr_delta_inv[0];
        Vec3 rot_vel = delta_vec[0] * dcoeff[1];
        for (auto i{1}; i < Degree; ++i) {
            d_vel_d_delta[i] =
                R_tmp[i - 1] * SO3::hat(rot_vel) * Jr_kdelta[i] * coeff[i + 1] +
                R_tmp[i] * dcoeff[i + 1];
            d_vel_d_delta[i] *= Jr_delta_inv[i];

            rot_vel = exp_k_delta[i] * rot_vel + delta_vec[i] * dcoeff[i + 1];
        }

        if (J) {
            J->start_idx = s;
            for (auto i{0}; i < Order; ++i) {
                J->d_val_d_knot[i].setZero();
            }
            for (auto i{0}; i < Degree; ++i) {
                J->d_val_d_knot[i] -= d_vel_d_delta[i];
                J->d_val_d_knot[i + 1] += d_vel_d_delta[i];
            }
        }

        return rot_vel;
    }

    /// @brief Evaluate rotational acceleration (second time derivative) of
    /// SO(3) B-spline in the body frame
    ///
    /// @param[in] time_ns time for evaluating acceleration of the spline in
    /// nanoseconds
    /// @param[out] vel_body if not nullptr, return the rotational velocity in
    /// the body frame (3x1 vector) (side computation)
    /// @return rotational acceleration (3x1 vector)
    Vec3 accelerationBody(int64_t time, Vec3* vel_body = nullptr) const
    {
        const auto [s, u] = internal::normalizeTime(time, _t0, _dt);

        VecN p = Base::baseCoeffsWithTime<0>(u);
        VecN coeff = this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<1>(u);
        VecN dcoeff = pow_inv_dt_[1] * this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<2>(u);
        VecN ddcoeff = pow_inv_dt_[2] * this->kCumulativeBlendingMatrix * p;

        SO3 r_accum;

        Vec3 rot_vel;
        rot_vel.setZero();

        Vec3 rot_accel;
        rot_accel.setZero();

        for (auto i{0}; i < Degree; ++i) {
            const SO3& p0 = _knots[s + i];
            const SO3& p1 = _knots[s + i + 1];

            SO3 r01 = p0.inverse() * p1;
            Vec3 delta = r01.log();

            SO3 rot = SO3::exp(-delta * coeff[i + 1]);

            rot_vel = rot * rot_vel;
            Vec3 vel_current = dcoeff[i + 1] * delta;
            rot_vel += vel_current;

            rot_accel = rot * rot_accel;
            rot_accel += ddcoeff[i + 1] * delta + rot_vel.cross(vel_current);
        }

        if (vel_body) {
            *vel_body = rot_vel;
        }
        return rot_accel;
    }

    /// @brief Evaluate rotational acceleration (second time derivative) of
    /// SO(3) B-spline in the body frame
    ///
    /// @param[in] time_ns time for evaluating acceleration of the spline in
    /// nanoseconds
    /// @param[out] J_accel if not nullptr, return the Jacobian of the
    /// rotational acceleration in body frame with respect to knots
    /// @param[out] vel_body if not nullptr, return the rotational velocity in
    /// the body frame (3x1 vector) (side computation)
    /// @param[out] J_vel if not nullptr, return the Jacobian of the rotational
    /// velocity in the body frame (side computation)
    /// @return rotational acceleration (3x1 vector)
    Vec3 accelerationBody(int64_t time, Jacobian* J_accel,
                          Vec3* vel_body = nullptr,
                          Jacobian* J_vel = nullptr) const
    {
        // BASALT_ASSERT(J_accel);

        const auto [s, u] = internal::normalizeTime(time, _t0, _dt);

        VecN p = Base::baseCoeffsWithTime<0>(u);
        const VecN coeff = this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<1>(u);
        const VecN dcoeff =
            pow_inv_dt_[1] * this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<2>(u);
        const VecN ddcoeff =
            pow_inv_dt_[2] * this->kCumulativeBlendingMatrix * p;

        Vec3 rot_vel;
        rot_vel.setZero();

        Vec3 rot_accel;
        rot_accel.setZero();

        Vec3 delta_vec[Degree];
        Mat3 exp_k_delta[Degree];
        Mat3 Jr_delta_inv[Degree];
        Mat3 Jr_kdelta[Degree];
        Vec3 rot_vel_arr[Degree];
        Vec3 rot_accel_arr[Degree];
        for (int i = 0; i < Degree; i++) {
            const SO3& p0 = _knots[s + i];
            const SO3& p1 = _knots[s + i + 1];

            SO3 r01 = p0.inverse() * p1;
            delta_vec[i] = r01.log();

            Sophus::rightJacobianInvSO3(delta_vec[i], Jr_delta_inv[i]);
            Jr_delta_inv[i] *= p1.inverse().matrix();

            Vec3 k_delta = coeff[i + 1] * delta_vec[i];
            Sophus::rightJacobianSO3(-k_delta, Jr_kdelta[i]);

            exp_k_delta[i] = Sophus::SO3d::exp(-k_delta).matrix();

            rot_vel = exp_k_delta[i] * rot_vel;
            Vec3 vel_current = dcoeff[i + 1] * delta_vec[i];
            rot_vel += vel_current;

            rot_accel = exp_k_delta[i] * rot_accel;
            rot_accel +=
                ddcoeff[i + 1] * delta_vec[i] + rot_vel.cross(vel_current);

            rot_vel_arr[i] = rot_vel;
            rot_accel_arr[i] = rot_accel;
        }

        Mat3 d_accel_d_delta[Degree];
        Mat3 d_vel_d_delta[Degree];
        d_vel_d_delta[Degree - 1] = coeff[Degree] * exp_k_delta[Degree - 1] *
                                        SO3::hat(rot_vel_arr[Degree - 2]) *
                                        Jr_kdelta[Degree - 1] +
                                    Mat3::Identity() * dcoeff[Degree];

        d_accel_d_delta[Degree - 1] =
            coeff[Degree] * exp_k_delta[Degree - 1] *
                SO3::hat(rot_accel_arr[Degree - 2]) * Jr_kdelta[Degree - 1] +
            Mat3::Identity() * ddcoeff[Degree] +
            dcoeff[Degree] *
                (SO3::hat(rot_vel_arr[Degree - 1]) -
                 SO3::hat(delta_vec[Degree - 1]) * d_vel_d_delta[Degree - 1]);

        Mat3 pj;
        pj.setIdentity();
        Vec3 sj;
        sj.setZero();
        for (int i = Degree - 2; i >= 0; i--) {
            sj += dcoeff[i + 2] * pj * delta_vec[i + 1];
            pj *= exp_k_delta[i + 1];

            d_vel_d_delta[i] = Mat3::Identity() * dcoeff[i + 1];
            if (i >= 1) {
                d_vel_d_delta[i] += coeff[i + 1] * exp_k_delta[i] *
                                    SO3::hat(rot_vel_arr[i - 1]) * Jr_kdelta[i];
            }

            d_accel_d_delta[i] =
                Mat3::Identity() * ddcoeff[i + 1] +
                dcoeff[i + 1] * (SO3::hat(rot_vel_arr[i]) -
                                 SO3::hat(delta_vec[i]) * d_vel_d_delta[i]);
            if (i >= 1) {
                d_accel_d_delta[i] += coeff[i + 1] * exp_k_delta[i] *
                                      SO3::hat(rot_accel_arr[i - 1]) *
                                      Jr_kdelta[i];
            }

            d_vel_d_delta[i] = pj * d_vel_d_delta[i];
            d_accel_d_delta[i] =
                pj * d_accel_d_delta[i] - SO3::hat(sj) * d_vel_d_delta[i];
        }

        if (J_vel) {
            J_vel->start_idx = s;
            for (auto i{0}; i < Order; i++) {
                J_vel->d_val_d_knot[i].setZero();
            }
            for (auto i{0}; i < Degree; i++) {
                Mat3 val = d_vel_d_delta[i] * Jr_delta_inv[i];

                J_vel->d_val_d_knot[i] -= val;
                J_vel->d_val_d_knot[i + 1] += val;
            }
        }

        if (J_accel) {
            J_accel->start_idx = s;
            for (auto i{0}; i < Order; ++i) {
                J_accel->d_val_d_knot[i].setZero();
            }

            for (auto i{0}; i < Degree; ++i) {
                Mat3 val = d_accel_d_delta[i] * Jr_delta_inv[i];

                J_accel->d_val_d_knot[i] -= val;
                J_accel->d_val_d_knot[i + 1] += val;
            }
        }

        if (vel_body) {
            *vel_body = rot_vel;
        }
        return rot_accel;
    }

    /// @brief Evaluate rotational jerk (third time derivative) of SO(3)
    /// B-spline in the body frame
    ///
    /// @param[in] time_ns time for evaluating jerk of the spline in
    /// nanoseconds
    /// @param[out] vel_body if not nullptr, return the rotational velocity in
    /// the body frame (3x1 vector) (side computation)
    /// @param[out] accel_body if not nullptr, return the rotational
    /// acceleration in the body frame (3x1 vector) (side computation)
    /// @return rotational jerk (3x1 vector)
    Vec3 jerkBody(int64_t time, Vec3* vel_body = nullptr,
                  Vec3* accel_body = nullptr) const
    {
        const auto [s, u] = internal::normalizeTime(time, _t0, _dt);

        VecN p = Base::baseCoeffsWithTime<0>(u);
        VecN coeff = this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<1>(u);
        VecN dcoeff = pow_inv_dt_[1] * this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<2>(u);
        VecN ddcoeff = pow_inv_dt_[2] * this->kCumulativeBlendingMatrix * p;

        p = Base::baseCoeffsWithTime<3>(u);
        VecN dddcoeff = pow_inv_dt_[3] * this->kCumulativeBlendingMatrix * p;

        Vec3 rot_vel;
        rot_vel.setZero();
        Vec3 rot_accel;
        rot_accel.setZero();
        Vec3 rot_jerk;
        rot_jerk.setZero();
        for (int i = 0; i < Degree; i++) {
            const SO3& p0 = _knots[s + i];
            const SO3& p1 = _knots[s + i + 1];

            SO3 r01 = p0.inverse() * p1;
            Vec3 delta = r01.log();

            SO3 rot = SO3::exp(-delta * coeff[i + 1]);

            rot_vel = rot * rot_vel;
            Vec3 vel_current = dcoeff[i + 1] * delta;
            rot_vel += vel_current;

            rot_accel = rot * rot_accel;
            Vec3 rot_vel_cross_vel_current = rot_vel.cross(vel_current);
            rot_accel += ddcoeff[i + 1] * delta + rot_vel_cross_vel_current;

            rot_jerk = rot * rot_jerk;
            rot_jerk +=
                dddcoeff[i + 1] * delta +
                (ddcoeff[i + 1] * rot_vel + 2 * dcoeff[i + 1] * rot_accel -
                 dcoeff[i + 1] * rot_vel_cross_vel_current)
                    .cross(delta);
        }

        if (vel_body) {
            *vel_body = rot_vel;
        }
        if (accel_body) {
            *accel_body = rot_accel;
        }
        return rot_jerk;
    }

protected:
    // Use Cumulative blending matrix
    std::deque<SO3> _knots;
    std::array<_Scalar, 4> pow_inv_dt_; ///< Array with inverse powers of dt
    int64_t _t0, _dt;
};

} // namespace tl
