#pragma once

#include <cassert>

#include "kalmanfilterbase.hpp"
#include "measurementmodel.hpp"
#include "systemmodel.hpp"

namespace tl {
namespace kalman {

// Abstract Base for Unscented Kalman Filters
//
// Parameters:
//     StateType[template]: The vector-type of the system state (usually some
//     type derived from Kalman::Vector)
//
// References:
// [1] The square-root unscented Kalman filter for state and
// parameter-estimation, by Rudolph van der Merwe and Eric A. Wan
template <class StateType>
class UnscentedKalmanFilterBase : public KalmanFilterBase<StateType>
{
public:
    using Base = KalmanFilterBase<StateType>;
    using typename Base::T;
    using typename Base::State;

    template <class Measurement, template <class> class CovarianceBase>
    using MeasurementModelType =
        MeasurementModel<State, Measurement, CovarianceBase>;

    template <class Control, template <class> class CovarianceBase>
    using SystemModelType = SystemModel<State, Control, CovarianceBase>;

protected:
    //! The number of sigma points (depending on state dimensionality)
    inline static constexpr int SigmaPointCount =
        2 * State::RowsAtCompileTime + 1;

    //! Vector containg the sigma scaling weights
    using SigmaWeights = Vector<T, SigmaPointCount>;

    //! Matrix type containing the sigma state or measurement points
    template <class Type>
    using SigmaPoints = Matrix<T, Type::RowsAtCompileTime, SigmaPointCount>;

protected:
    //! State Estimate
    using Base::x;

    //! Sigma weights (m)
    SigmaWeights sigmaWeights_m;
    //! Sigma weights (c)
    SigmaWeights sigmaWeights_c;

    //! Sigma points (state)
    SigmaPoints<State> sigmaStatePoints;

    // Weight parameters
    T alpha; //!< Scaling parameter for spread of sigma points (usually \f$ 1E-4
             //!< \leq \alpha \leq 1 \f$)
    T beta; //!< Parameter for prior knowledge about the distribution (\f$ \beta
            //!< = 2 \f$ is optimal for Gaussian)
    T kappa;  //!< Secondary scaling parameter (usually 0)
    T gamma;  //!< \f$ \gamma = \sqrt{L + \lambda} \f$ with \f$ L \f$ being the
              //!< state dimensionality
    T lambda; //!< \f$ \lambda = \alpha^2 ( L + \kappa ) - L\f$ with \f$ L \f$
              //!< being the state dimensionality

protected:
    // See paper for more details
    //
    // Parameters:
    //     _alpha [in]: Scaling parameter for spread of sigma points (usually
    //     1e-4 <= alpha <= 1)
    //     _beta[in]: Parameter for prior knowledge about the distribution (beta
    //     = 2 is optimal for Gaussian)
    //     _kappa[in]: Secondary scaling parameter (usually 0)
    UnscentedKalmanFilterBase(T _alpha = T(1), T _beta = T(2), T _kappa = T(0))
        : alpha(_alpha), beta(_beta), kappa(_kappa)
    {
        // Pre-compute all weights
        computeWeights();

        // Setup state and covariance
        x.setZero();
    }

    /**
     * @brief Compute predicted state using system model and control input
     *
     * @param [in] s The System Model
     * @param [in] u The Control input
     * @return The predicted state
     */
    template <class Control, template <class> class CovarianceBase>
    State computeStatePrediction(
        const SystemModelType<Control, CovarianceBase>& s, const Control& u)
    {
        // Pass each sigma point through non-linear state transition function
        computeSigmaPointTransition(s, u);

        // Compute predicted state from predicted sigma points
        return computePredictionFromSigmaPoints<State>(sigmaStatePoints);
    }

    /**
     * @brief Compute predicted measurement using measurement model and
     * predicted sigma measurements
     *
     * @param [in] m The Measurement Model
     * @param [in] sigmaMeasurementPoints The predicted sigma measurement points
     * @return The predicted measurement
     */
    template <class Measurement, template <class> class CovarianceBase>
    Measurement computeMeasurementPrediction(
        const MeasurementModelType<Measurement, CovarianceBase>& m,
        SigmaPoints<Measurement>& sigmaMeasurementPoints)
    {
        // Predict measurements for each sigma point
        computeSigmaPointMeasurements<Measurement>(m, sigmaMeasurementPoints);

        // Predict measurement from sigma measurement points
        return computePredictionFromSigmaPoints<Measurement>(
            sigmaMeasurementPoints);
    }

    /**
     * @brief Compute sigma weights
     */
    void computeWeights()
    {
        T L = T(State::RowsAtCompileTime);
        lambda = alpha * alpha * (L + kappa) - L;
        gamma = std::sqrt(L + lambda);

        // Make sure L != -lambda to avoid division by zero
        assert(std::abs(L + lambda) > 1e-6);

        // Make sure L != -kappa to avoid division by zero
        assert(std::abs(L + kappa) > 1e-6);

        T W_m_0 = lambda / (L + lambda);
        T W_c_0 = W_m_0 + (T(1) - alpha * alpha + beta);
        T W_i = T(1) / (T(2) * alpha * alpha * (L + kappa));

        // Make sure W_i > 0 to avoid square-root of negative number
        assert(W_i > T(0));

        sigmaWeights_m[0] = W_m_0;
        sigmaWeights_c[0] = W_c_0;

        for (int i = 1; i < SigmaPointCount; ++i) {
            sigmaWeights_m[i] = W_i;
            sigmaWeights_c[i] = W_i;
        }
    }

    /**
     * @brief Predict expected sigma states from current sigma states using
     * system model and control input
     *
     * @note This covers equation (18) of Algorithm 3.1 in the Paper
     *
     * @param [in] s The System Model
     * @param [in] u The Control input
     */
    template <class Control, template <class> class CovarianceBase>
    void computeSigmaPointTransition(
        const SystemModelType<Control, CovarianceBase>& s, const Control& u)
    {
        for (int i = 0; i < SigmaPointCount; ++i) {
            sigmaStatePoints.col(i) = s.f(sigmaStatePoints.col(i), u);
        }
    }

    /**
     * @brief Predict the expected sigma measurements from predicted sigma
     * states using measurement model
     *
     * @note This covers equation (23) of Algorithm 3.1 in the Paper
     *
     * @param [in] m The Measurement model
     * @param [out] sigmaMeasurementPoints The struct of expected sigma
     * measurements to be computed
     */
    template <class Measurement, template <class> class CovarianceBase>
    void computeSigmaPointMeasurements(
        const MeasurementModelType<Measurement, CovarianceBase>& m,
        SigmaPoints<Measurement>& sigmaMeasurementPoints)
    {
        for (int i = 0; i < SigmaPointCount; ++i) {
            sigmaMeasurementPoints.col(i) = m.h(sigmaStatePoints.col(i));
        }
    }

    /**
     * @brief Compute state or measurement prediciton from sigma points using
     * pre-computed sigma weights
     *
     * @note This covers equations (19) and (24) of Algorithm 3.1 in the Paper
     *
     * @param [in] sigmaPoints The computed sigma points of the desired type
     * (state or measurement)
     * @return The prediction
     */
    template <class Type>
    Type computePredictionFromSigmaPoints(const SigmaPoints<Type>& sigmaPoints)
    {
        // Use efficient matrix x vector computation
        return sigmaPoints * sigmaWeights_m;
    }
};

} // namespace kalman
} // namespace tl
