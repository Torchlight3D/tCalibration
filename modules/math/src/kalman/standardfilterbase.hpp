#pragma once

#include "types.hpp"

namespace tl {
namespace kalman {

// Abstract base class for standard (non-square root) filters and models
//
// Parameters:
//     StateType[template]: The vector-type of the system state (usually some
//     type derived from Kalman::Vector)
template <class StateType>
class StandardBase
{
public:
    const Covariance<StateType>& getCovariance() const { return P; }

    bool setCovariance(const Covariance<StateType>& covariance)
    {
        P = covariance;
        return true;
    }

    CovarianceSquareRoot<StateType> getCovarianceSquareRoot() const
    {
        return CovarianceSquareRoot<StateType>(P);
    }

    /**
     * @brief Set Covariance using Square Root
     *
     * @param covSquareRoot Lower triangular Matrix representing the covariance
     *                      square root (i.e. P = LLˆT)
     */
    bool setCovarianceSquareRoot(const Covariance<StateType>& covSquareRoot)
    {
        CovarianceSquareRoot<StateType> S;
        S.setL(covSquareRoot);
        P = S.reconstructedMatrix();
        return true;
    }

protected:
    StandardBase() { P.setIdentity(); }

protected:
    Covariance<StateType> P;
};

// Abstract base class for standard (non-square root) filters
//
// Parameters:
//     StateType[template]: The vector-type of the system state (usually some
//     type derived from Kalman::Vector)
template <class StateType>
class StandardFilterBase : public StandardBase<StateType>
{
protected:
    using Base = StandardBase<StateType>;

    using Base::P;
};

} // namespace kalman
} // namespace tl
