#pragma once

namespace tl {
namespace kalman {

// Abstract base class for all Kalman Filters
//
// Parameters:
//     StateType[template]: The vector-type of the system state (usually some
//     type derived from Kalman::Vector)
template <class StateType>
class KalmanFilterBase
{
    static_assert(StateType::RowsAtCompileTime > 0,
                  "State vector must contain at least 1 element");
    static_assert(StateType::ColsAtCompileTime == 1,
                  "State type must be a column vector");

public:
    using T = typename StateType::Scalar;
    using State = StateType;

    virtual ~KalmanFilterBase() = default;

public:
    const State& getState() const { return x; }

    void init(const State& initialState) { x = initialState; }

protected:
    KalmanFilterBase() {}

protected:
    State x;
};

} // namespace kalman
} // namespace tl
