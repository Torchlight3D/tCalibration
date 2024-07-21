#pragma once

#include <format>

#include <Eigen/Cholesky>
#include <glog/logging.h>

#include "sparsecholeskyllt.h"

namespace tl {

namespace internal {

// These are template-like overrides that allow the sparse linear solvers to
// work with sparse or dense matrices.
inline void Compute(const Eigen::SparseMatrix<double>& spmat,
                    SparseCholeskyLLt* solver)
{
    solver->compute(spmat);
}

inline void Compute(const Eigen::MatrixXd& mat, SparseCholeskyLLt* solver)
{
    solver->compute(mat.sparseView());
}

inline Eigen::VectorXd Shrinkage(const Eigen::VectorXd& vec, double kappa)
{
    Eigen::ArrayXd zeros(vec.size());
    zeros.setZero();
    return zeros.max(vec.array() - kappa) - zeros.max(-vec.array() - kappa);
}

} // namespace internal

// Brief:
// An L1 norm approximation solver that attempts to solve the problem:
//                      || A * x - b ||_1
// This problem can be solved with the alternating direction method of
// multipliers (ADMM) as a least unsquared deviations minimizer.
//
// ADMM can be much faster than interior point methods but convergence may be
// slower. Generally speaking, ADMM solvers converge to good solutions in only a
// few number of iterations, but can spend many iterations subsuquently refining
// the solution to optain the global optimum. The speed improvements are because
// the matrix A only needs to be factorized (by Cholesky decomposition) once, as
// opposed to every iteration.
//
// Ref:
// "Distributed Optimization and Statistical Learning via the Alternating
// Direction Method of Multipliers" by Boyd et al (Foundations and Trends in
// Machine Learning 2012).
struct L1SolverOptions
{
    int max_num_iterations = 1000;

    // Rho is the augmented Lagrangian parameter.
    double rho = 1.;

    // Alpha is the over-relaxation parameter, typically [1.0 1.8].
    double alpha = 1.;

    // Stop criteria
    double absolute_tolerance = 1e-4;
    double relative_tolerance = 1e-2;
};

template <class Matrix_t>
class L1Solver
{
public:
    L1Solver(const L1SolverOptions& options, const Matrix_t& mat)
        : _opts(options), _A(mat)
    {
        // Analyze the sparsity pattern once. Only the values of the entries
        // will be changed with each iteration.
        const Matrix_t AtA = _A.transpose() * _A;
        internal::Compute(AtA, &_solver);
        CHECK_EQ(_solver.info(), Eigen::Success);
    }

    void SetMaxIterations(int maxIterations)
    {
        _opts.max_num_iterations = maxIterations;
    }

    // Solves ||Ax - b||_1 for the optimial L1 solution given an initial guess
    // for x. To solve this we introduce an auxillary variable y such that the
    // solution to:
    //        min   1 * y
    //   s.t. [  A   -I ] [ x ] < [  b ]
    //        [ -A   -I ] [ y ]   [ -b ]
    // which is an equivalent linear program.
    Eigen::VectorXd Solve(const Eigen::VectorXd& b)
    {
        using Eigen::VectorXd;

        VectorXd x;
        VectorXd z(_A.rows());
        VectorXd u(_A.rows());
        z.setZero();
        u.setZero();

        VectorXd Ax(_A.rows());
        VectorXd z_old(z.size());
        VectorXd Ax_hat(_A.rows());

        // Precompute some convergence terms.
        const double rhs_norm = b.norm();
        const double primal_abs_tolerance_eps =
            std::sqrt(_A.rows()) * _opts.absolute_tolerance;
        const double dual_abs_tolerance_eps =
            std::sqrt(_A.cols()) * _opts.absolute_tolerance;

        VLOG(2) << std::format("{:^10}{:^10}{:^10}{:^10}{:^10}", "Iteration",
                               "R norm", "S norm", "Primal eps", "Dual eps");

        for (int i = 0; i < _opts.max_num_iterations; i++) {
            // Update x.
            x.noalias() = _solver.solve(_A.transpose() * (b + z - u));
            if (_solver.info() != Eigen::Success) {
                LOG(ERROR) << "L1 Minimization failed. "
                              "Could not solve the sparse linear system with "
                              "Cholesky Decomposition";
                return {};
            }

            Ax.noalias() = _A * x;
            Ax_hat.noalias() = _opts.alpha * Ax;
            Ax_hat.noalias() += (1. - _opts.alpha) * (z + b);

            // Update z and set z_old.
            std::swap(z, z_old);
            z.noalias() = internal::Shrinkage(Ax_hat - b + u, 1. / _opts.rho);

            // Update u.
            u.noalias() += Ax_hat - z - b;

            // Compute the convergence terms.
            const double r_norm = (Ax - z - b).norm();
            const double s_norm =
                (-_opts.rho * _A.transpose() * (z - z_old)).norm();
            const double max_norm = std::max({Ax.norm(), z.norm(), rhs_norm});
            const double primal_eps =
                primal_abs_tolerance_eps + _opts.relative_tolerance * max_norm;
            const double dual_eps = dual_abs_tolerance_eps +
                                    _opts.relative_tolerance *
                                        (_opts.rho * _A.transpose() * u).norm();

            VLOG(2) << std::format("{:4d}{:4.4e}{:4.4e}{:4.4e}{:4.4e}", i,
                                   r_norm, s_norm, primal_eps, dual_eps);

            // Determine if the minimizer has converged.
            if (r_norm < primal_eps && s_norm < dual_eps) {
                break;
            }
        }

        return x;
    }

private:
    L1SolverOptions _opts;
    Matrix_t _A;
    SparseCholeskyLLt _solver;
};

} // namespace tl
