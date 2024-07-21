#pragma once

#include <Eigen/Core>

#include "bcmsdpsolver.h"

namespace tl::math {

// This algorithm actually solves the SDP problem:
//           min. -tr(RY), s.t. Y_{ii} = I_3, i=1, ..., n, Y \succeq 0
// and the optimal solution could be retrieved by reading the first three rows
// of Y^*
//
// Ref:
// [1] "Row by row methods for semidefinite programming." by Z. Wen, D.
// Goldfarb, S. Ma, and K. Scheinberg. (Technical report, Columbia University,
// 2009. 7)
// [2] "Rotation averaging and strong duality" by Eriksson A, Olsson C, Kahl F,
// et al. (CVPR 2018)
class RBRSDPSolver : public BCMSDPSolver
{
public:
    RBRSDPSolver(size_t n, size_t block_dim, const Options& options = {});

    Summary Solve() override;

    Eigen::MatrixXd GetSolution() const override { return X_; }

    double EvaluateFuncVal() const override;
    double EvaluateFuncVal(const Eigen::MatrixXd& Y) const override;

private:
    void ReformingB(size_t k, Eigen::MatrixXd& Bk) const;
    void ReformingW(size_t k, Eigen::MatrixXd& Wk) const;
    void ReorderingUnknown(size_t k, const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& W);

private:
    Eigen::MatrixXd X_;
};

} // namespace tl::math
