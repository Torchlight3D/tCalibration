#include "rbrsdpsolver.h"

#include <limits>

#include <ceres/rotation.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <tMath/Eigen/Utils>

namespace tl::math {

using Eigen::Matrix3d;
using Eigen::MatrixXd;

RBRSDPSolver::RBRSDPSolver(size_t n, size_t block_dim, const Options& options)
    : BCMSDPSolver(n, block_dim, options)
{
    X_ = MatrixXd::Identity(dim_ * n, dim_ * n);
}

SDPSolver::Summary RBRSDPSolver::Solve()
{
    double prev_func_val = std::numeric_limits<double>::max();
    double cur_func_val = this->EvaluateFuncVal();
    double duration = 0.0;
    double error = 0.0;

    Summary summary;
    summary.begin_time = std::chrono::high_resolution_clock::now();

    while (summary.total_iterations_num < sdp_solver_options_.max_iterations) {
        LogToStd(summary.total_iterations_num, prev_func_val, cur_func_val,
                 error, duration);

        if (IsConverge(prev_func_val, cur_func_val,
                       sdp_solver_options_.tolerance, &error)) {
            break;
        }

        // convergence rate? Take it for consideration.
        for (size_t k = 0; k < n_; k++) {
            // Eliminating the k-th row and column from Y to form Bk
            MatrixXd B = MatrixXd::Zero(3 * (n_ - 1), 3 * (n_ - 1));
            ReformingB(k, B);

            // Eliminating the k-th column and all but the k-th row from R to
            // form Wk
            MatrixXd W = MatrixXd::Zero(3 * (n_ - 1), 3);
            ReformingW(k, W);

            MatrixXd BW = B * W;
            MatrixXd WtBW = W.transpose() * BW;

            // FIXME: (chenyu) Solving matrix square root with
            // SVD and LDL^T would generate different result
            MatrixXd sqrt_WtBW = MatrixSquareRoot(WtBW);
            // Eigen::MatrixXd WtBW_sqrt =
            // MatrixSquareRootForSemidefinitePositiveMat(WtBW);

            // FIXME: (chenyu) Eigen 3.3.0 is required for the use of
            // CompleteOrthogonalDecomposition<>
            // Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>
            // cqr(WtBW_sqrt); Eigen::Matrix3d moore_penrose_pseinv =
            // cqr.pseudoInverse();
            Matrix3d moore_penrose_pseinv = sqrt_WtBW.inverse();

            // compute S by fixing the error of Equ.(47) in Erikson's paper
            MatrixXd S = -BW * moore_penrose_pseinv;

            // reordering X
            ReorderingUnknown(k, B, S);
        }

        summary.total_iterations_num++;
        duration = summary.Duration();

        // Update function value
        prev_func_val = cur_func_val;
        cur_func_val = this->EvaluateFuncVal();
    }

    summary.total_iterations_num++;

    LogToStd(summary.total_iterations_num, prev_func_val, cur_func_val, error,
             duration);

    return summary;
}

double RBRSDPSolver::EvaluateFuncVal() const { return EvaluateFuncVal(X_); }

double RBRSDPSolver::EvaluateFuncVal(const Eigen::MatrixXd& Y) const
{
    return (Q_ * Y).trace();
}

void RBRSDPSolver::ReformingB(size_t k, Eigen::MatrixXd& B) const
{
    // The row index of matrix B
    size_t r{0};
    for (size_t i{0}; i < n_; ++i) {
        if (i == k) {
            continue;
        }

        // The col index of matrix B
        size_t c{0};
        for (size_t j{0}; j < n_; ++j) {
            if (j == k) {
                continue;
            }

            B.block(3 * r, 3 * c, 3, 3) = X_.block(3 * i, 3 * j, 3, 3);
            c++;
        }
        r++;
    }
}

void RBRSDPSolver::ReformingW(size_t k, Eigen::MatrixXd& W) const
{
    // The row index of matrix W
    size_t r{0};
    for (size_t i{0}; i < n_; ++i) {
        if (i == k) {
            continue;
        }

        W.block(3 * r, 0, 3, 3) = Q_.block(3 * i, 3 * k, 3, 3);
        r++;
    }
}

// Reordering X according to [Algorithm 1] in paper
void RBRSDPSolver::ReorderingUnknown(size_t k, const Eigen::MatrixXd& B,
                                     const Eigen::MatrixXd& S)
{
    // Update X(k, k)
    X_.block(3 * k, 3 * k, 3, 3) = Matrix3d::Identity();

    // Update the k-th column of Y, except Y(k, k)
    // the j-th block of S
    size_t j = 0;
    for (size_t i = 0; i < n_; i++) {
        if (i == k) {
            continue;
        }

        X_.block(3 * i, 3 * k, 3, 3) = S.block(3 * j, 0, 3, 3);
        j++;
    }

    // Update the k-th row of Y, except Y(k, k)
    // the i-th block of S
    size_t i = 0;
    for (size_t j = 0; j < n_; j++) {
        if (j == k) {
            continue;
        }

        X_.block(3 * k, 3 * j, 3, 3) = S.block(3 * i, 0, 3, 3).transpose();
        i++;
    }
}

} // namespace tl::math
