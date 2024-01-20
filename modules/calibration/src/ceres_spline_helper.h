#pragma once

#include <ceres/ceres.h>

namespace tl {

// Brief:
// Helper for implementing Lie group and Euclidean B-splines of order N by ceres
template <class T, int _N>
struct CeresSplineHelper
{
    static constexpr int N = _N;       // Order of the spline.
    static constexpr int DEG = _N - 1; // Degree of the spline.

    using MatN = Eigen::Matrix<T, _N, _N>;
    using VecN = Eigen::Matrix<T, _N, 1>;

    static const MatN blending_matrix_;
    static const MatN cumulative_blending_matrix_;
    static const MatN base_coefficients_;

    // Brief:
    // Vector of derivatives of time polynomial.
    //
    // Explanation:
    // Computes a derivative of \f$ \begin{bmatrix}1 & t & t^2 & \dots &
    // t^{N-1}\end{bmatrix} \f$ with repect to time. For example, the first
    // derivative would be \f$ \begin{bmatrix}0 & 1 & 2 t & \dots & (N-1)
    // t^{N-2}\end{bmatrix} \f$.
    //
    // Template Params:
    //     Derivative: derivative to evaluate
    // Input:
    //     t:
    // Output:
    //     res_const: vector to store the result
    template <int Derivative, class Derived>
    static inline void baseCoeffsWithTime(
        const Eigen::MatrixBase<Derived>& res_const, T t)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, N);
        auto& res = const_cast<Eigen::MatrixBase<Derived>&>(res_const);

        res.setZero();

        if (Derivative < N) {
            res[Derivative] = base_coefficients_(Derivative, Derivative);

            T _t = t;
            for (int j = Derivative + 1; j < N; j++) {
                res[j] = base_coefficients_(Derivative, j) * _t;
                _t = _t * t;
            }
        }
    }

    /// Brief:
    /// Evaluate Lie group cummulative B-spline and time derivatives.
    ///
    /// @param[in] sKnots array of pointers of the spline knots. The size of
    /// each knot should be GroupT::num_parameters: 4 for SO(3) and 7 for SE(3).
    /// @param[in] u normalized time to compute value of the spline
    /// @param[in] inv_dt inverse of the time spacing in seconds between spline
    /// knots
    /// @param[out] transform if not nullptr return the value of the spline
    /// @param[out] vel if not nullptr velocity (first time derivative) in
    /// the body frame
    /// @param[out] accel if not nullptr acceleration (second time
    /// derivative) in the body frame
    template <template <class> class GroupT>
    static inline void evaluate_lie(
        const T* const* sKnots, const T u, const T inv_dt,
        GroupT<T>* transform = nullptr,
        typename GroupT<T>::Tangent* vel = nullptr,
        typename GroupT<T>::Tangent* accel = nullptr,
        typename GroupT<T>::Tangent* jerk = nullptr)
    {
        using Group = GroupT<T>;
        using Tangent = typename GroupT<T>::Tangent;
        using Adjoint = typename GroupT<T>::Adjoint;

        VecN p, coeff, dcoeff, ddcoeff, dddcoeff;

        CeresSplineHelper<T, N>::template baseCoeffsWithTime<0>(p, u);
        coeff = CeresSplineHelper<T, N>::cumulative_blending_matrix_ * p;

        if (vel || accel || jerk) {
            CeresSplineHelper<T, N>::template baseCoeffsWithTime<1>(p, u);
            dcoeff = inv_dt *
                     CeresSplineHelper<T, N>::cumulative_blending_matrix_ * p;

            if (accel || jerk) {
                CeresSplineHelper<T, N>::template baseCoeffsWithTime<2>(p, u);
                ddcoeff = inv_dt * inv_dt *
                          CeresSplineHelper<T, N>::cumulative_blending_matrix_ *
                          p;

                if (jerk) {
                    CeresSplineHelper<T, N>::template baseCoeffsWithTime<3>(p,
                                                                            u);
                    dddcoeff =
                        inv_dt * inv_dt * inv_dt *
                        CeresSplineHelper<T, N>::cumulative_blending_matrix_ *
                        p;
                }
            }
        }

        if (transform) {
            Eigen::Map<const Group> const p00(sKnots[0]);
            *transform = p00;
        }

        Tangent rot_vel, rot_accel, rot_jerk;

        if (vel || accel || jerk)
            rot_vel.setZero();
        if (accel || jerk)
            rot_accel.setZero();
        if (jerk)
            rot_jerk.setZero();

        for (int i = 0; i < DEG; i++) {
            Eigen::Map<const Group> const p0(sKnots[i]);
            Eigen::Map<const Group> const p1(sKnots[i + 1]);

            Group r01 = p0.inverse() * p1;
            Tangent delta = r01.log();

            Group exp_kdelta = Group::exp(delta * coeff[i + 1]);

            if (transform)
                (*transform) *= exp_kdelta;

            if (vel || accel || jerk) {
                Adjoint A = exp_kdelta.inverse().Adj();

                rot_vel = A * rot_vel;
                Tangent rot_vel_current = delta * dcoeff[i + 1];
                rot_vel += rot_vel_current;

                if (accel || jerk) {
                    rot_accel = A * rot_accel;
                    Tangent accel_lie_bracket =
                        Group::lieBracket(rot_vel, rot_vel_current);
                    rot_accel += ddcoeff[i + 1] * delta + accel_lie_bracket;

                    if (jerk) {
                        rot_jerk = A * rot_jerk;
                        rot_jerk += dddcoeff[i + 1] * delta +
                                    Group::lieBracket(
                                        ddcoeff[i + 1] * rot_vel +
                                            T(2) * dcoeff[i + 1] * rot_accel -
                                            dcoeff[i + 1] * accel_lie_bracket,
                                        delta);
                    }
                }
            }
        }

        if (vel)
            *vel = rot_vel;
        if (accel)
            *accel = rot_accel;
        if (jerk)
            *jerk = rot_jerk;
    }

    // Brief:
    // Evaluate Euclidean B-spline or time derivatives.
    //
    // Input:
    //     sKnots: array of pointers of the spline knots. The size of each knot
    //     should be DIM.
    //     u: normalized time to compute value of the spline
    //     inv_dt: inverse of the time spacing in seconds between spline knots
    // Output:
    //     vec: if DERIV=0 returns value of the spline, otherwise corresponding
    //     derivative.
    template <int DIM, int DERIV>
    static inline void evaluate(const T* const* sKnots, const T u,
                                const T inv_dt, Eigen::Matrix<T, DIM, 1>* vec)
    {
        if (!vec)
            return;

        using VecD = Eigen::Matrix<T, DIM, 1>;

        VecN p, coeff;

        CeresSplineHelper<T, N>::template baseCoeffsWithTime<DERIV>(p, u);
        coeff = ceres::pow(inv_dt, DERIV) *
                CeresSplineHelper<T, N>::blending_matrix_ * p;

        vec->setZero();

        for (int i = 0; i < N; i++) {
            Eigen::Map<const VecD> const p(sKnots[i]);

            (*vec) += coeff[i] * p;
        }
    }
};

/// Brief:
/// Compute base coefficient matrix for polynomials of size N.
///
/// Explanation:
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
/// Template Params:
///     _N: order of the polynomial
///     _Scalar: scalar type to use
template <int Order_t, typename T = double>
Eigen::Matrix<T, Order_t, Order_t> computeBaseCoefficients()
{
    Eigen::Matrix<double, Order_t, Order_t> base_coefficients;
    base_coefficients.setZero();
    base_coefficients.row(0).setOnes();

    constexpr int kDegree = Order_t - 1;

    int order = kDegree;
    for (int n{1}; n < Order_t; n++) {
        for (int i = kDegree - order; i < Order_t; i++) {
            base_coefficients(n, i) =
                (order - kDegree + i) * base_coefficients(n - 1, i);
        }
        order--;
    }

    return base_coefficients.template cast<T>();
}

template <class T, int _N>
const typename CeresSplineHelper<T, _N>::MatN
    CeresSplineHelper<T, _N>::base_coefficients_ =
        computeBaseCoefficients<_N, T>();

// Compute binomial coefficient.
inline constexpr uint64_t C_n_k(uint64_t n, uint64_t k)
{
    if (k > n) {
        return 0;
    }

    uint64_t res{1};
    for (uint64_t d{1}; d <= k; ++d) {
        res *= n--;
        res /= d;
    }
    return res;
}

/// Brief:
/// Compute blending matrix for uniform B-spline evaluation.
///
/// Template Params:
///     _N: order of the spline
///     _Scalar: scalar type to use
///     _Cumulative: if the spline should be cumulative
template <int Order_t, typename T = double, bool _Cumulative = false>
Eigen::Matrix<T, Order_t, Order_t> computeBlendingMatrix()
{
    Eigen::Matrix<double, Order_t, Order_t> m;
    m.setZero();

    for (int i{0}; i < Order_t; ++i) {
        for (int j{0}; j < Order_t; ++j) {
            double sum{0.};
            for (int s = j; s < Order_t; ++s) {
                sum += std::pow(-1.0, s - j) * C_n_k(Order_t, s - j) *
                       std::pow(Order_t - s - 1.0, Order_t - i - 1.0);
            }

            m(j, i) = C_n_k(Order_t - 1, Order_t - 1 - i) * sum;
        }
    }

    if constexpr (_Cumulative) {
        for (int i{0}; i < Order_t; i++) {
            for (int j = i + 1; j < Order_t; j++) {
                m.row(i) += m.row(j);
            }
        }
    }

    uint64_t factorial{1};
    for (int i{2}; i < Order_t; ++i) {
        factorial *= i;
    }

    return (m / factorial).template cast<T>();
}

template <class T, int _N>
const typename CeresSplineHelper<T, _N>::MatN
    CeresSplineHelper<T, _N>::blending_matrix_ =
        computeBlendingMatrix<_N, T, false>();

template <class T, int _N>
const typename CeresSplineHelper<T, _N>::MatN
    CeresSplineHelper<T, _N>::cumulative_blending_matrix_ =
        computeBlendingMatrix<_N, T, true>();

} // namespace tl
