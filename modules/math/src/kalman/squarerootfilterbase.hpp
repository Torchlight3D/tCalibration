#pragma once

#include "types.hpp"

namespace tl {
namespace kalman {

/**
 * @brief Abstract base class for square-root filters and models
 *
 * @param StateType The vector-type of the system state (usually some type
 * derived from Kalman::Vector)
 */
template <class StateType>
class SquareRootBase
{
protected:
    //! Covariance Square Root
    CovarianceSquareRoot<StateType> S;

public:
    const CovarianceSquareRoot<StateType>& getCovarianceSquareRoot() const
    {
        return S;
    }

    bool setCovariance(const Covariance<StateType>& covariance)
    {
        S.compute(covariance);
        return (S.info() == Eigen::Success);
    }

    Covariance<StateType> getCovariance() const
    {
        return S.reconstructedMatrix();
    }

    /**
     * @brief Set Covariance using Square Root
     *
     * @param covSquareRoot Lower triangular Matrix representing the covariance
     *                      square root (i.e. P = LLˆT)
     */
    bool setCovarianceSquareRoot(const Covariance<StateType>& covSquareRoot)
    {
        S.setL(covSquareRoot);
        return true;
    }

protected:
    SquareRootBase() { S.setIdentity(); }
};

/**
 * @brief Abstract base class for square root filters
 *
 * @param StateType The vector-type of the system state (usually some type
 * derived from Kalman::Vector)
 */
template <class StateType>
class SquareRootFilterBase : public SquareRootBase<StateType>
{
protected:
    using Base = SquareRootBase<StateType>;

    using Base::S;
};

} // namespace kalman
} // namespace tl
