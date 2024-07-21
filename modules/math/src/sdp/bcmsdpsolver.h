#pragma once

#include <iomanip>
#include <iostream>

#include <Eigen/Core>

#include "sdpsolver.h"

namespace tl::math {

// Semidefinite positive solver with Block Coordinate Minimization(BCM)
class BCMSDPSolver : public SDPSolver
{
public:
    using SDPSolver::SDPSolver;
    virtual ~BCMSDPSolver() = default;

protected:
    // Judge if algorithm converges
    virtual bool IsConverge(double prev_funv, double cur_funv, double tolerance,
                            double* error) const
    {
        *error =
            std::abs(prev_funv - cur_funv) / std::max(std::abs(prev_funv), 1.);
        return *error <= tolerance;
    }

    // FIXME: Remove this
    void LogToStd(size_t iter, double pre_funv, double cur_funv, double error,
                  double time)
    {
        if (iter < 1) {
            std::cout << "\n"
                      << std::setw(11) << std::setfill(' ') << "Iter "
                      << std::setw(16) << std::setfill(' ') << "Prev "
                      << std::setw(16) << std::setfill(' ') << "Cur "
                      << std::setw(16) << std::setfill(' ') << "Error "
                      << std::setw(16) << std::setfill(' ') << "Time"
                      << std::endl;
        }
        else {
            std::cout << std::setw(8) << std::setfill(' ') << iter
                      << std::setw(5) << std::setfill(' ') << " "
                      << std::setw(14) << std::setfill(' ') << pre_funv
                      << std::setw(5) << std::setfill(' ') << " "
                      << std::setw(12) << std::setfill(' ') << cur_funv
                      << std::setw(5) << std::setfill(' ') << " "
                      << std::setw(12) << std::setfill(' ') << error
                      << std::setw(2) << std::setfill(' ') << " "
                      << std::setw(10) << std::setfill(' ') << time
                      << std::endl;
        }
    }
};

} // namespace tl::math
