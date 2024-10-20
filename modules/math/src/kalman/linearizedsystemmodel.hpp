#pragma once

#include "systemmodel.hpp"

namespace tl {
namespace kalman {

template <class StateType>
class ExtendedKalmanFilter;

template <class StateType>
class SquareRootExtendedKalmanFilter;

/**
 * @brief Abstract base class of all linearized (first order taylor expansion)
 * system models
 *
 * @param StateType The vector-type of the system state (usually some type
 * derived from Kalman::Vector)
 * @param ControlType The vector-type of the control input (usually some type
 * derived from Kalman::Vector)
 * @param CovarianceBase The class template used for covariance storage (must be
 * either StandardBase or SquareRootBase)
 */
template <class StateType,
          class ControlType = Vector<typename StateType::Scalar, 0>,
          template <class> class CovarianceBase = StandardBase>
class LinearizedSystemModel
    : public SystemModel<StateType, ControlType, CovarianceBase>
{
    friend class ExtendedKalmanFilter<StateType>;
    friend class SquareRootExtendedKalmanFilter<StateType>;

public:
    using Base = SystemModel<StateType, ControlType, CovarianceBase>;

    using typename Base::State;

    using typename Base::Control;

protected:
    //! System model jacobian
    Jacobian<State, State> F;
    //! System model noise jacobian
    Jacobian<State, State> W;

    /**
     * Callback function for state-dependent update of Jacobi-matrices F and W
     * before each update step
     */
    virtual void updateJacobians(const State& x, const Control& u)
    {
        // No update by default
        (void)x;
        (void)u;
    }

protected:
    LinearizedSystemModel()
    {
        F.setIdentity();
        W.setIdentity();
    }
    ~LinearizedSystemModel() {}
};

} // namespace kalman
} // namespace tl
