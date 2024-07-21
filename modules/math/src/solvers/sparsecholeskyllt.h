#pragma once

#include <Eigen/SparseCholesky>

namespace tl {

// Brief:
// A class for performing the choleksy decomposition of a sparse matrix using
// CHOLMOD from SuiteSparse. This allows us to utilize the supernodal algorithms
// which are not included with Eigen. CHOLMOD automatically determines if the
// simplicial or supernodal algorithm is the best choice.
//
// NOTE:
// 1. The matrix mat should be a symmetric matrix.
// 2. We could use CHOLMOD, which is 3-4 faster for big problem, but it's under
// GPL. Here Eigen::SimplisticalLDLT is used instead.
class SparseCholeskyLLt
{
public:
    SparseCholeskyLLt();
    explicit SparseCholeskyLLt(const Eigen::SparseMatrix<double>& mat);

    // Perform symbolic analysis of the matrix. This is useful for analyzing
    // matrices with the same sparsity pattern when used in conjunction with
    // factorize().
    void analyzePattern(const Eigen::SparseMatrix<double>& mat);

    // Perform numerical decomposition of the current matrix. If the matrix has
    // the same sparsity pattern as the previous decomposition then this method
    // may be used to efficiently decompose the matrix by avoiding symbolic
    // analysis.
    void factorize(const Eigen::SparseMatrix<double>& mat);

    // Computes the Cholesky decomposition of mat. This is the same as calling
    // analyzePattern() followed by factorize().
    void compute(const Eigen::SparseMatrix<double>& mat);

    // Returns the current state of the decomposition. After each step users
    // should ensure that info() returns Eigen::Success.
    Eigen::ComputationInfo info() const;

    // Using the cholesky decomposition, solve for x that minimizes
    //                        lhs * x = rhs
    // where lhs is the factorized matrix.
    Eigen::VectorXd solve(const Eigen::VectorXd& rhs);

private:
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Upper> _solver;
    Eigen::ComputationInfo _info;
    bool _factorized;
    bool _analyzed;
};

} // namespace tl
