#pragma once

#include "kalmanfilterbase.hpp"
#include "standardfilterbase.hpp"
#include "linearizedmeasurementmodel.hpp"
#include "linearizedsystemmodel.hpp"

namespace tl {
namespace kalman {

// Extended Kalman Filter (EKF)
//
// Parameters:
//     StateType[template]: The vector-type of the system state (usually some
//     type derived from Kalman::Vector)
template <class StateType>
class ExtendedKalmanFilter : public KalmanFilterBase<StateType>,
                             public StandardFilterBase<StateType>
{
public:
    using KalmanBase = KalmanFilterBase<StateType>;
    using StandardBase = StandardFilterBase<StateType>;

    //! Numeric Scalar Type inherited from base
    using typename KalmanBase::T;

    //! State Type inherited from base
    using typename KalmanBase::State;

    //! Linearized Measurement Model Type
    template <class Measurement, template <class> class CovarianceBase>
    using MeasurementModelType =
        LinearizedMeasurementModel<State, Measurement, CovarianceBase>;

    //! Linearized System Model Type
    template <class Control, template <class> class CovarianceBase>
    using SystemModelType =
        LinearizedSystemModel<State, Control, CovarianceBase>;

protected:
    //! Kalman Gain Matrix Type
    template <class Measurement>
    using KalmanGain = KalmanGain<State, Measurement>;

protected:
    //! State Estimate
    using KalmanBase::x;
    //! State Covariance Matrix
    using StandardBase::P;

public:
    ExtendedKalmanFilter()
    {
        // Setup state and covariance
        P.setIdentity();
    }

    /**
     * @brief Perform filter prediction step using system model and no control
     * input (i.e. \f$ u = 0 \f$)
     *
     * @param [in] s The System model
     * @return The updated state estimate
     */
    template <class Control, template <class> class CovarianceBase>
    const State& predict(SystemModelType<Control, CovarianceBase>& s)
    {
        // predict state (without control)
        Control u;
        u.setZero();
        return predict(s, u);
    }

    /**
     * @brief Perform filter prediction step using control input \f$u\f$ and
     * corresponding system model
     *
     * @param [in] s The System model
     * @param [in] u The Control input vector
     * @return The updated state estimate
     */
    template <class Control, template <class> class CovarianceBase>
    const State& predict(SystemModelType<Control, CovarianceBase>& s,
                         const Control& u)
    {
        s.updateJacobians(x, u);

        // predict state
        x = s.f(x, u);

        // predict covariance
        P = (s.F * P * s.F.transpose()) +
            (s.W * s.getCovariance() * s.W.transpose());

        // return state prediction
        return this->getState();
    }

    /**
     * @brief Perform filter update step using measurement \f$z\f$ and
     * corresponding measurement model
     *
     * @param [in] m The Measurement model
     * @param [in] z The measurement vector
     * @return The updated state estimate
     */
    template <class Measurement, template <class> class CovarianceBase>
    const State& update(MeasurementModelType<Measurement, CovarianceBase>& m,
                        const Measurement& z)
    {
        m.updateJacobians(x);

        // COMPUTE KALMAN GAIN
        // compute innovation covariance
        Covariance<Measurement> S = (m.H * P * m.H.transpose()) +
                                    (m.V * m.getCovariance() * m.V.transpose());

        // compute kalman gain
        KalmanGain<Measurement> K = P * m.H.transpose() * S.inverse();

        // UPDATE STATE ESTIMATE AND COVARIANCE
        // Update state using computed kalman gain and innovation
        x += K * (z - m.h(x));

        // Update covariance
        P -= K * m.H * P;

        // return updated state estimate
        return this->getState();
    }
};

} // namespace kalman
} // namespace tl
