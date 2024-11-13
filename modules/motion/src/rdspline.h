#pragma once

#include <deque>

#include <Eigen/Core>

#include "splinebase.h"

namespace tl {

/// @brief Uniform B-spline for euclidean vectors with dimention DIM of order
/// N
///
/// For example, in the particular case scalar values and order N=5, for a time
/// \f$t \in [t_i, t_{i+1})\f$ the value of \f$p(t)\f$ depends only on 5 control
/// points at \f$[t_i, t_{i+1}, t_{i+2}, t_{i+3}, t_{i+4}]\f$. To
/// simplify calculations we transform time to uniform representation \f$s(t) =
/// (t - t_0)/\Delta t \f$, such that control points transform into \f$ s_i \in
/// [0,..,N] \f$. We define function \f$ u(t) = s(t)-s_i \f$ to be a time since
/// the start of the segment. Following the matrix representation of De Boor -
/// Cox formula, the value of the function can be
/// evaluated as follows: \f{align}{
///    p(u(t)) &=
///    \begin{pmatrix} p_{i}\\ p_{i+1}\\ p_{i+2}\\ p_{i+3}\\ p_{i+4}
///    \end{pmatrix}^T M_5 \begin{pmatrix} 1 \\ u \\ u^2 \\ u^3 \\ u^4
///    \end{pmatrix},
/// \f}
/// where \f$ p_{i} \f$ are knots and  \f$ M_5 \f$ is a blending matrix computed
/// using \ref computeBlendingMatrix \f{align}{
///    M_5 = \frac{1}{4!}
///    \begin{pmatrix} 1 & -4 & 6 & -4 & 1 \\ 11 & -12  & -6 & 12  & -4 \\11 &
///    12 &  -6 &  -12  &  6 \\ 1  &  4  &  6  &  4  & -4 \\ 0  &  0  &  0  &  0
///    &  1 \end{pmatrix}.
/// \f}
/// Given this formula, we can evaluate derivatives with respect to time
/// (velocity, acceleration) in the following way:
/// \f{align}{
///    p'(u(t)) &= \frac{1}{\Delta t}
///    \begin{pmatrix} p_{i}\\ p_{i+1}\\ p_{i+2}\\ p_{i+3}\\ p_{i+4}
///    \end{pmatrix}^T
///    M_5
///    \begin{pmatrix} 0 \\ 1 \\ 2u \\ 3u^2 \\ 4u^3 \end{pmatrix},
/// \f}
/// \f{align}{
///    p''(u(t)) &= \frac{1}{\Delta t^2}
///    \begin{pmatrix} p_{i}\\ p_{i+1}\\ p_{i+2}\\ p_{i+3}\\ p_{i+4}
///    \end{pmatrix}^T
///    M_5
///    \begin{pmatrix} 0 \\ 0 \\ 2 \\ 6u \\ 12u^2 \end{pmatrix}.
/// \f}
/// Higher time derivatives are evaluated similarly. This class supports
/// vector values for knots \f$ p_{i} \f$. The corresponding derivative vector
/// on the right is computed using \ref baseCoeffsWithTime.
///
/// See [[arXiv:1911.08860]](https://arxiv.org/abs/1911.08860) for more details.
template <int _Dim, int _Order, std::floating_point _Scalar = double>
class RdSpline : public SplineBase<_Order, _Scalar>
{
    using Base = SplineBase<_Order, _Scalar>;

public:
    using Scalar = Base::Scalar;

    enum
    {
        Dim = _Dim,
        Order = Base::Order,
        Degree = Base::Degree,
    };

    // TODO: Remove this
    static constexpr _Scalar NS_TO_S = 1e-9;
    static constexpr _Scalar S_TO_NS = 1e9;

    using MatN = Base::MatN;
    using VecN = Base::VecN;

    using MatD = Eigen::Matrix<Scalar, Dim, Dim>;
    using VecD = Eigen::Vector<Scalar, Dim>;

    /// @brief Struct to store the Jacobian of the spline
    ///
    /// Since B-spline of order N has local support (only N knots infuence the
    /// value) the Jacobian is zero for all knots except maximum N for value and
    /// all derivatives.
    struct Jacobian
    {
        ///< Start index of the non-zero elements of the Jacobian.
        size_t start_idx;

        ///< Value of nonzero Jacobians.
        std::array<Scalar, Order> d_val_d_knot;
    };

    RdSpline() = default;
    explicit RdSpline(int64_t dt, int64_t start = 0) : _t0(start), _dt(dt)
    {
        pow_inv_dt_[0] = 1.0;
        pow_inv_dt_[1] = S_TO_NS / _dt;
        for (auto i{2}; i < Order; i++) {
            pow_inv_dt_[i] = pow_inv_dt_[i - 1] * pow_inv_dt_[1];
        }
    }

    template <typename Scalar2>
    inline RdSpline<Dim, Order, Scalar2> cast() const
    {
        RdSpline<Dim, Order, Scalar2> other{_dt, _t0};

        for (auto i{0}; i < _Order; i++) {
            other.pow_inv_dt_[i] = pow_inv_dt_[i];
        }

        for (const auto& knot : _knots) {
            other._knots.emplace_back(knot.template cast<Scalar2>());
        }

        return other;
    }

    /// @brief Gererate random trajectory
    ///
    /// @param[in] n number of knots to generate
    /// @param[in] static_init if true the first N knots will be the same
    /// resulting in static initial condition
    void setRandom(int n, bool static_init = false)
    {
        if (static_init) {
            VecD rnd = VecD::Random() * 5;

            for (int i = 0; i < Order; i++) {
                _knots.push_back(rnd);
            }
            for (int i = 0; i < n - Order; i++) {
                _knots.push_back(VecD::Random() * 5);
            }
        }
        else {
            for (int i = 0; i < n; i++) {
                _knots.push_back(VecD::Random() * 5);
            }
        }
    }

    int64_t timeInterval() const { return _dt; }
    int64_t startTime() const { return _t0; }
    inline void setStartTime(int64_t t0) { _t0 = t0; }
    int64_t endTime() const
    {
        return _t0 + (_knots.size() - Order + 1) * _dt - 1;
    }

    inline const VecD& frontKnot() const { return _knots.front(); }
    inline VecD& rKnot(int i) { return _knots[i]; }
    inline const VecD& knot(int i) const { return _knots[i]; }
    const std::deque<VecD>& knots() const { return _knots; }

    inline void pushBack(const VecD& knot) { _knots.push_back(knot); }
    inline void popFront()
    {
        _t0 += _dt;
        _knots.pop_front();
    }
    inline void popBack() { _knots.pop_back(); }

    inline void resize(size_t n) { _knots.resize(n); }

    /// @brief Evaluate value or derivative of the spline
    ///
    /// @param _Derivative derivative to evaluate (0 for value)
    /// @param[in] time_ns time for evaluating of the spline in nanoseconds
    /// @param[out] J if not nullptr, return the Jacobian of the value with
    /// respect to knots
    /// @return value of the spline or derivative. Euclidean vector of dimention
    /// DIM.
    template <int _Derivative = 0>
    VecD evaluate(int64_t time, Jacobian* J = nullptr) const
    {
        const auto [size, u] = internal::normalizeTime(time, _t0, _dt);

        VecN p = Base::baseCoeffsWithTime<_Derivative>(u);
        const VecN coeff =
            pow_inv_dt_[_Derivative] * (this->kBlendingMatrix * p);

        VecD res;
        res.setZero();
        for (auto i{0}; i < Order; i++) {
            res += coeff[i] * _knots[size + i];

            if (J) {
                J->d_val_d_knot[i] = coeff[i];
            }
        }

        if (J) {
            J->start_idx = size;
        }

        return res;
    }

    inline VecD velocity(int64_t time, Jacobian* J = nullptr) const
    {
        return evaluate<1>(time, J);
    }

    inline VecD acceleration(int64_t time, Jacobian* J = nullptr) const
    {
        return evaluate<2>(time, J);
    }

protected:
    // Use NonCumulative blending matrix
    std::deque<VecD> _knots;
    std::array<Scalar, Order> pow_inv_dt_;
    int64_t _t0{0}, _dt{0};
};

} // namespace tl
