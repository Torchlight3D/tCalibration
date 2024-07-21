#pragma once

#include <memory>

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "sdpsolver.h"

namespace tl::math {

class RankRestrictedSDPSolver;

class RiemannianStaircase : public SDPSolver
{
public:
    struct Options
    {
        size_t min_rank = 3;
        size_t max_rank = 10;

        size_t max_eigen_solver_iterations = 20;

        double min_eigenvalue_nonnegativity_tolerance = 1e-5;

        size_t num_Lanczos_vectors = 20;

        SDPSolverType local_solver_type = SDPSolverType::RANK_DEFICIENT_BCM;

        double gradient_tolerance = 1e-2;

        double preconditioned_gradient_tolerance = 1e-4;

        Options() {}
    };

    RiemannianStaircase(size_t n, size_t block_dim, const Options& options = {},
                        const SDPSolver::Options& sdpOptions = {});

    Summary Solve() override;

    Eigen::MatrixXd GetSolution() const override;

    double EvaluateFuncVal() const override;
    double EvaluateFuncVal(const Eigen::MatrixXd& Y) const override;

    void SetCovariance(const Eigen::SparseMatrix<double>& Q) override;
    void SetAdjacentEdges(const std::unordered_map<size_t, std::vector<size_t>>&
                              adj_edges) override;

private:
    bool KKTVerification(double* min_eigenvalue,
                         Eigen::VectorXd* min_eigenvector,
                         size_t* num_iterations);

    bool EscapeSaddle(double lambda_min, const Eigen::VectorXd& vector_min,
                      double gradient_tolerance,
                      double preconditioned_gradient_tolerance,
                      Eigen::MatrixXd* Yplus);

    void RoundSolution();

private:
    size_t n_;
    size_t dim_;

    Options _opts;

    std::shared_ptr<RankRestrictedSDPSolver> sdp_solver_;

    Eigen::MatrixXd R_;
};

} // namespace tl::math
