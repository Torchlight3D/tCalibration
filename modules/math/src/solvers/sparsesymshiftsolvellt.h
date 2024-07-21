#pragma once

#include <Eigen/SparseCore>

#include <glog/logging.h>

#include "sparsecholeskyllt.h"

namespace tl {

// Brief:
// A sparse method for computing the shift and inverse linear operator.
//
// NOTE:
// 1. This method is intended for use with the Spectra library.
template <typename Scalar_>
struct SparseSymShiftSolveLLT
{
    using Scalar = Scalar_;

    const Eigen::SparseMatrix<double>& _mat;
    std::shared_ptr<SparseCholeskyLLt> _solver;
    double sigma_;

    SparseSymShiftSolveLLT(std::shared_ptr<SparseCholeskyLLt> solver,
                           const Eigen::SparseMatrix<double>& mat)
        : _solver(solver), _mat(mat)
    {
        CHECK_EQ(_mat.rows(), _mat.cols());

        _solver->compute(_mat);
        if (_solver->info() != Eigen::Success) {
            LOG(FATAL)
                << "Failed to perform Cholesky decomposition on the matrix."
                   "Make sure the input matrix is positive semi-definite.";
        }
    }

    size_t rows() const { return _mat.rows(); }
    size_t cols() const { return _mat.cols(); }

    void set_shift(double sigma) { sigma_ = sigma; }

    // Use LDLT to perform matrix inversion on the positive semidefinite matrix.
    void perform_op(const double* x_in, double* y_out) const
    {
        Eigen::Map<const Eigen::VectorXd> x(x_in, _mat.rows());
        Eigen::Map<Eigen::VectorXd> y(y_out, _mat.cols());
        y = _solver->solve(x);
        if (_solver->info() != Eigen::Success) {
            LOG(FATAL)
                << "Failed to perform Cholesky decomposition on the matrix."
                   "Make sure the input matrix is positive semi-definite.";
        }
    }
};

} // namespace tl
