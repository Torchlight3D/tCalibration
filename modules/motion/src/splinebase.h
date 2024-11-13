#pragma once

#include <Eigen/Core>

namespace tl {

namespace internal {
/// @brief Compute binomial coefficient.
///
/// Computes number of combinations that include k objects out of n.
/// @param[in] n
/// @param[in] k
/// @return binomial coefficient
inline constexpr uint64_t binomialCoefficient(uint64_t n, uint64_t k)
{
    if (k > n) {
        return 0;
    }

    uint64_t r = 1;
    for (uint64_t d = 1; d <= k; ++d) {
        r *= n--;
        r /= d;
    }
    return r;
}

// Return <Size, NormalizedTime>
inline std::tuple<int64_t, double> normalizeTime(int64_t time, int64_t t0,
                                                 int64_t dt)
{
    const int64_t duration = (time - t0);
    int64_t size = duration / dt;
    double u = double(duration % dt) / double(dt);
    return {size, u};
}

/// @brief Compute blending matrix for uniform B-spline evaluation.
///
/// @param _N order of the spline
/// @param _Scalar scalar type to use
/// @param _Cumulative if the spline should be cumulative
template <int _Order, typename _Scalar = double, bool _Cumulative = false>
Eigen::Matrix<_Scalar, _Order, _Order> computeBlendingMatrix()
{
    Eigen::Matrix<double, _Order, _Order> m;
    m.setZero();
    for (int i = 0; i < _Order; ++i) {
        for (int j = 0; j < _Order; ++j) {
            double sum = 0;
            for (int s = j; s < _Order; ++s) {
                sum += std::pow(-1.0, s - j) *
                       binomialCoefficient(_Order, s - j) *
                       std::pow(_Order - s - 1.0, _Order - 1.0 - i);
            }
            m(j, i) = binomialCoefficient(_Order - 1, _Order - 1 - i) * sum;
        }
    }

    if constexpr (_Cumulative) {
        for (int i = 0; i < _Order; i++) {
            for (int j = i + 1; j < _Order; j++) {
                m.row(i) += m.row(j);
            }
        }
    }

    uint64_t factorial = 1;
    for (int i = 2; i < _Order; ++i) {
        factorial *= i;
    }

    return (m / factorial).template cast<_Scalar>();
}

/// @brief Compute base coefficient matrix for polynomials of size N.
///
/// In each row starting from 0 contains the derivative coefficients of the
/// polynomial. For _N=5 we get the following matrix: \f[ \begin{bmatrix}
///   1 & 1 & 1 & 1 & 1
/// \\0 & 1 & 2 & 3 & 4
/// \\0 & 0 & 2 & 6 & 12
/// \\0 & 0 & 0 & 6 & 24
/// \\0 & 0 & 0 & 0 & 24
/// \\ \end{bmatrix}
/// \f]
/// Functions \ref RdSpline::baseCoeffsWithTime and \ref
/// So3Spline::baseCoeffsWithTime use this matrix to compute derivatives of the
/// time polynomial.
///
/// @param _N order of the polynomial
/// @param _Scalar scalar type to use
template <int _Order, typename _Scalar = double>
Eigen::Matrix<_Scalar, _Order, _Order> computeBaseCoefficients()
{
    Eigen::Matrix<double, _Order, _Order> base_coefficients;
    base_coefficients.setZero();
    base_coefficients.row(0).setOnes();

    constexpr int DEG = _Order - 1;
    int order = DEG;
    for (int n = 1; n < _Order; n++) {
        for (int i = DEG - order; i < _Order; i++) {
            base_coefficients(n, i) =
                (order - DEG + i) * base_coefficients(n - 1, i);
        }
        order--;
    }
    return base_coefficients.template cast<_Scalar>();
}

} // namespace internal

template <int _Order, std::floating_point _Scalar = double>
class SplineBase
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

    /// @brief Evaluate Lie group cummulative B-spline and time derivatives.
    ///
    /// @param[in] sKnots array of pointers of the spline knots. The size of
    /// each knot should be _LieGroup::num_parameters: 4 for SO(3) and 7 for
    /// SE(3).
    /// @param[in] u normalized time to compute value of the spline
    /// @param[in] inv_dt inverse of the time spacing in seconds between spline
    /// knots
    /// @param[out] transform_out if not nullptr return the value of the spline
    /// @param[out] vel_out if not nullptr velocity (first time derivative) in
    /// the body frame
    /// @param[out] accel_out if not nullptr acceleration (second time
    /// derivative) in the body frame
    template <class T, template <class, int = 0> class _LieGroup>
    inline static void evaluate_lie(
        const T* const* knots, T u, T inv_dt,
        _LieGroup<T>* transform_out = nullptr,
        typename _LieGroup<T>::Tangent* vel_out = nullptr,
        typename _LieGroup<T>::Tangent* accel_out = nullptr,
        typename _LieGroup<T>::Tangent* jerk_out = nullptr)
    {
        using LieGroup = _LieGroup<T>;
        using Tangent = typename LieGroup::Tangent;
        using Adjoint = typename LieGroup::Adjoint;

        using _VecN = Eigen::Vector<T, Order>;

        _VecN p, coeff, dcoeff, ddcoeff, dddcoeff;
        p = baseCoeffsWithTime<0>(u);
        coeff = kCumulativeBlendingMatrix * p;

        if (vel_out || accel_out || jerk_out) {
            p = baseCoeffsWithTime<1>(u);
            dcoeff = inv_dt * kCumulativeBlendingMatrix * p;

            if (accel_out || jerk_out) {
                p = baseCoeffsWithTime<2>(u);
                ddcoeff = inv_dt * inv_dt * kCumulativeBlendingMatrix * p;

                if (jerk_out) {
                    p = baseCoeffsWithTime<3>(u);
                    dddcoeff = inv_dt * inv_dt * inv_dt *
                               kCumulativeBlendingMatrix * p;
                }
            }
        }

        if (transform_out) {
            Eigen::Map<LieGroup const> const p00(knots[0]);
            *transform_out = p00;
        }

        Tangent rot_vel, rot_accel, rot_jerk;
        if (vel_out || accel_out || jerk_out) {
            rot_vel.setZero();
        }
        if (accel_out || jerk_out) {
            rot_accel.setZero();
        }
        if (jerk_out) {
            rot_jerk.setZero();
        }

        for (auto i{0}; i < Degree; ++i) {
            Eigen::Map<LieGroup const> const p0(knots[i]);
            Eigen::Map<LieGroup const> const p1(knots[i + 1]);

            LieGroup r01 = p0.inverse() * p1;
            Tangent delta = r01.log();

            LieGroup exp_kdelta = LieGroup::exp(delta * coeff[i + 1]);

            if (transform_out)
                (*transform_out) *= exp_kdelta;

            if (vel_out || accel_out || jerk_out) {
                Adjoint A = exp_kdelta.inverse().Adj();

                rot_vel = A * rot_vel;
                Tangent rot_vel_current = delta * dcoeff[i + 1];
                rot_vel += rot_vel_current;

                if (accel_out || jerk_out) {
                    rot_accel = A * rot_accel;
                    Tangent accel_lie_bracket =
                        LieGroup::lieBracket(rot_vel, rot_vel_current);
                    rot_accel += ddcoeff[i + 1] * delta + accel_lie_bracket;

                    if (jerk_out) {
                        rot_jerk = A * rot_jerk;
                        rot_jerk += dddcoeff[i + 1] * delta +
                                    LieGroup::lieBracket(
                                        ddcoeff[i + 1] * rot_vel +
                                            T(2) * dcoeff[i + 1] * rot_accel -
                                            dcoeff[i + 1] * accel_lie_bracket,
                                        delta);
                    }
                }
            }
        }

        if (vel_out) {
            *vel_out = rot_vel;
        }
        if (accel_out) {
            *accel_out = rot_accel;
        }
        if (jerk_out) {
            *jerk_out = rot_jerk;
        }
    }

    /// @brief Evaluate Euclidean B-spline or time derivatives.
    ///
    /// @param[in] sKnots array of pointers of the spline knots. The size of
    /// each knot should be DIM.
    /// @param[in] u normalized time to compute value of the spline
    /// @param[in] inv_dt inverse of the time spacing in seconds between spline
    /// knots
    /// @param[out] vec_out if _Derivative=0 returns value of the spline,
    /// otherwise corresponding derivative.
    template <class T, int _Dim, int _Derivative>
    inline static void evaluate(const T* const* knots, T u, T inv_dt,
                                Eigen::Vector<T, _Dim>* vec_out)
    {
        if (!vec_out) {
            return;
        }

        using VecD = Eigen::Vector<T, _Dim>;

        using _VecN = Eigen::Vector<T, Order>;

        _VecN p = baseCoeffsWithTime<_Derivative>(u);
        const _VecN coeff = pow(inv_dt, _Derivative) * kBlendingMatrix * p;

        vec_out->setZero();
        for (auto i{0}; i < Order; ++i) {
            const Eigen::Map<const VecD> p{knots[i]};
            (*vec_out) += coeff[i] * p;
        }
    }

protected:
    /// @brief Vector of derivatives of time polynomial.
    ///
    /// Computes a derivative of \f$ \begin{bmatrix}1 & t & t^2 & \dots &
    /// t^{N-1}\end{bmatrix} \f$ with repect to time. For example, the first
    /// derivative would be \f$ \begin{bmatrix}0 & 1 & 2 t & \dots & (N-1)
    /// t^{N-2}\end{bmatrix} \f$.
    /// @param _Derivative derivative to evaluate
    /// @param[out] res_const vector to store the result
    /// @param[in] t
    template <int _Derivative, typename T = double>
    inline static Eigen::Vector<T, Order> baseCoeffsWithTime(T t)
    {
        static_assert(_Derivative < Order);

        const auto _baseCoefficients = kBaseCoefficients.template cast<T>();
        using _VecN = Eigen::Vector<T, Order>;
        _VecN res;
        res.setZero();

        res[_Derivative] = _baseCoefficients(_Derivative, _Derivative);

        auto _t = t;
        for (auto j = _Derivative + 1; j < Order; j++) {
            res[j] = _baseCoefficients(_Derivative, j) * _t;
            _t = _t * t;
        }

        return res;
    }

    inline static const MatN kBlendingMatrix =
        internal::computeBlendingMatrix<Order, Scalar>();
    inline static const MatN kCumulativeBlendingMatrix =
        internal::computeBlendingMatrix<Order, Scalar, true>();
    inline static const MatN kBaseCoefficients =
        internal::computeBaseCoefficients<Order>();
};

} // namespace tl
