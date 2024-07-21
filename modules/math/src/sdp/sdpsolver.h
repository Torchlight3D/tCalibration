#pragma once

#include <chrono>
#include <iostream>
#include <unordered_map>

#include <Eigen/SparseCore>

namespace tl {

namespace math {

enum SDPSolverType
{
    RBR_BCM,
    RANK_DEFICIENT_BCM,
    RIEMANNIAN_STAIRCASE,
    SE_SYNC, // not implemented
    SHONAN   // not implemented
};

enum PreconditionerType
{
    None,
    JACOBI,
    INCOMPLETE_CHOLESKY,
    REGULARIZED_CHOLESKY
};

// Brief:
// Semidefinite programming solver
class SDPSolver
{
public:
    struct Options
    {
        // maximum iteration number
        size_t max_iterations = 500;

        // tolerance for convergence
        double tolerance = 1e-8;

        // Not implemented
        PreconditionerType preconditioner_type = PreconditionerType::None;

        int num_threads = 8;

        Options() {}
    };

    SDPSolver(size_t n, size_t block_dim, const Options& opts = {})
        : n_(n), dim_(block_dim), sdp_solver_options_(opts)
    {
        Q_ = Eigen::SparseMatrix<double>(dim_ * n, dim_ * n);
    }

    virtual ~SDPSolver() = default;

    void SetSolverOptions(const Options& options)
    {
        sdp_solver_options_ = options;
    }

    virtual void SetCovariance(const Eigen::SparseMatrix<double>& Q) { Q_ = Q; }

    virtual void SetAdjacentEdges(
        const std::unordered_map<size_t, std::vector<size_t>>& adj_edges)
    {
        adj_edges_ = adj_edges;
    }

    size_t NumUnknowns() const { return n_; }
    size_t Dimension() const { return dim_; }

    struct Summary
    {
        unsigned total_iterations_num = 0;

        std::chrono::high_resolution_clock::time_point begin_time;
        std::chrono::high_resolution_clock::time_point end_time;
        std::chrono::high_resolution_clock::time_point prev_time;

        Summary() {}

        double TotalTime() const
        {
            return std::chrono::duration_cast<
                       std::chrono::duration<int, std::milli>>(end_time -
                                                               begin_time)
                .count();
        }

        double Duration()
        {
            if (total_iterations_num == 1) {
                prev_time = begin_time;
            }
            else {
                prev_time = end_time;
            }

            end_time = std::chrono::high_resolution_clock::now();
            return std::chrono::duration_cast<
                       std::chrono::duration<int, std::milli>>(end_time -
                                                               prev_time)
                .count();
        }

        void Report()
        {
            std::cout << "\n"
                         "Lagrange Dual Rotation Averaging Report: "
                         "\n"
                         "Total iterations: "
                      << total_iterations_num
                      << "\n"
                         "Time took: "
                      << TotalTime()
                      << " milliseconds"
                         "\n";
        }
    };

    virtual Summary Solve() = 0;
    virtual Eigen::MatrixXd GetSolution() const = 0;
    virtual double EvaluateFuncVal() const = 0;
    virtual double EvaluateFuncVal(const Eigen::MatrixXd& Y) const = 0;

protected:
    // number of unknown blocks.
    size_t n_;

    // the dimension of matrix sub-block.
    size_t dim_;

    // covariance matrix in SDP problem: min tr(QX).
    Eigen::SparseMatrix<double> Q_;

    std::unordered_map<size_t, std::vector<size_t>> adj_edges_;

    Options sdp_solver_options_;
};

} // namespace math

} // namespace tl
