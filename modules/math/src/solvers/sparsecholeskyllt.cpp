#include "sparsecholeskyllt.h"

#include <glog/logging.h>

namespace tl {

SparseCholeskyLLt::SparseCholeskyLLt()
    : _factorized(false), _analyzed(false), _info(Eigen::Success)
{
}

SparseCholeskyLLt::SparseCholeskyLLt(const Eigen::SparseMatrix<double>& mat)
    : _factorized(false), _analyzed(false), _info(Eigen::Success)
{
    compute(mat);
}

void SparseCholeskyLLt::analyzePattern(const Eigen::SparseMatrix<double>& mat)
{
    _solver.analyzePattern(mat);
    _info = _solver.info();
    _analyzed = _info == Eigen::Success;
    _factorized = false;
    _info = Eigen::Success;
}

void SparseCholeskyLLt::factorize(const Eigen::SparseMatrix<double>& mat)
{
    _solver.factorize(mat);
    _info = _solver.info();
    _factorized = _info == Eigen::Success;
}

void SparseCholeskyLLt::compute(const Eigen::SparseMatrix<double>& mat)
{
    _solver.compute(mat);
    _info = _solver.info();
    if (_info == Eigen::Success) {
        _factorized = true;
        _analyzed = true;
    }
}

Eigen::ComputationInfo SparseCholeskyLLt::info() const { return _info; }

Eigen::VectorXd SparseCholeskyLLt::solve(const Eigen::VectorXd& rhs)
{
    CHECK(_analyzed) << "Cannot call Solve() because symbolic analysis "
                        "of the matrix (i.e. AnalyzePattern()) failed!";
    CHECK(_factorized) << "Cannot call Solve() because numeric factorization "
                          "of the matrix (i.e. Factorize()) failed!";

    return _solver.solve(rhs);
}

} // namespace tl
