#pragma once

#include <type_traits>

#include "standardfilterbase.hpp"
#include "types.hpp"

namespace tl {
namespace kalman {
/**
 * @brief Abstract base class of all system models
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
class SystemModel : public CovarianceBase<StateType>
{
    static_assert(StateType::RowsAtCompileTime > 0,
                  "State vector must contain at least 1 element");
    static_assert(ControlType::RowsAtCompileTime >= 0,
                  "Control vector must contain at least 0 elements");
    static_assert(std::is_same_v<typename StateType::Scalar,
                                 typename ControlType::Scalar>,
                  "State and Control scalar types must be identical");

public:
    using State = StateType;

    using Control = ControlType;

public:
    /**
     * @brief State Transition Function f
     *
     * Computes the predicted system state in the next timestep given
     * the current state x and the control input u
     */
    virtual State f(const State& x, const Control& u) const = 0;

protected:
    SystemModel() {}
    virtual ~SystemModel() {}
};

} // namespace kalman
} // namespace tl
