#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>

#include "bcmsdpsolver.h"

namespace tl::math {

// This algorithm has superior efficiency than the normal BCMSDPSolver in large
// scale SDP optimization problems. Instead of solving the original SDP problem:
// -----------------------------------------------------------
//                  min. tr(QX)
//                  s.t. X[i,i] = I_d, \forall i \in [n],
//                       X \succeq 0
// -----------------------------------------------------------
// This algorithm solves for the rank-restricted SDP problem:
// -----------------------------------------------------------
//                       min. tr(QY^TY)
//                       s.t. Y \in St(d,r)^n
// -----------------------------------------------------------
// where St(d,r)^n = {Y = [Y1 Y2 ... Yn] \in R^{rxdn}: Yi \in St(d,r)},
// and the Stiefel manifold St(d,r) = {Y \in R^{rxd}: Y^TY=I_d}
//
class RankRestrictedSDPSolver : public BCMSDPSolver
{
public:
    RankRestrictedSDPSolver(size_t n, size_t block_dim,
                            const Options& options = {});

    Summary Solve() override;

    Eigen::MatrixXd GetSolution() const override;

    double EvaluateFuncVal() const override;
    double EvaluateFuncVal(const Eigen::MatrixXd& Y) const override;

    void SetOptimalY(const Eigen::MatrixXd& Y);

    Eigen::MatrixXd ComputeLambdaMatrix() const;
    const Eigen::MatrixXd& GetYStar() const;
    const Eigen::MatrixXd ComputeQYt(const Eigen::MatrixXd& Y) const;

    void AugmentRank();

    size_t CurrentRank() const;

    Eigen::MatrixXd Project(const Eigen::MatrixXd& A) const;
    Eigen::MatrixXd Retract(const Eigen::MatrixXd& Y,
                            const Eigen::MatrixXd& V) const;

    Eigen::MatrixXd EuclideanGradient(const Eigen::MatrixXd& Y) const;
    Eigen::MatrixXd RiemannianGradient(const Eigen::MatrixXd& Y,
                                       const Eigen::MatrixXd& nablaF_Y) const;
    Eigen::MatrixXd RiemannianGradient(const Eigen::MatrixXd& Y) const;

    Eigen::MatrixXd TangentSpaceProjection(const Eigen::MatrixXd& Y,
                                           const Eigen::MatrixXd& dotY) const;
    Eigen::MatrixXd SymBlockDiagProduct(const Eigen::MatrixXd& A,
                                        const Eigen::MatrixXd& B,
                                        const Eigen::MatrixXd& C) const;

    Eigen::MatrixXd Precondition(const Eigen::MatrixXd& Y,
                                 const Eigen::MatrixXd& dotY) const;

private:
    // The rank-deficient first-order critical point
    // with row-block size is rank_*dim_.
    Eigen::MatrixXd Y_;

    // initial rank of Y_.
    size_t rank_;
};

} // namespace tl::math
