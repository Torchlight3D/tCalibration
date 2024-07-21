#include "linearrotationestimator.h"

#include <unordered_map>

#include <ceres/rotation.h>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <Spectra/SymEigsShiftSolver.h>

#include <tCamera/CameraMatrixUtils>
#include <tCore/ContainerUtils>
#include <tMath/Solvers/SparseSymShiftSolveLLT>

namespace tl {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

using SparseMatrixd = Eigen::SparseMatrix<double>;
using Tripletd = Eigen::Triplet<double>;

namespace {

constexpr int kNumRotationMatrixDimensions = 3;

void Fill3x3SparseMatrix(const Eigen::Matrix3d& mat, int row, int col,
                         std::vector<Eigen::Triplet<double>>* triplets)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Only add the value if it is not zero.
            triplets->emplace_back(row + i, col + j, mat(i, j));
        }
    }
}

} // namespace

bool LinearRotationEstimator::EstimateRotations(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* orientations)
{
    for (const auto& [pair, info] : viewPairs) {
        AddRelativeRotationConstraint(pair, info.rotation);
    }

    return EstimateRotations(orientations);
}

void LinearRotationEstimator::AddRelativeRotationConstraint(
    const ViewIdPair& viewPairId, const Eigen::Vector3d& relative_rotation)
{
    // Add a new view-id to sparse matrix index mapping. This is a no-op if the
    // view has already been assigned to a matrix index.
    con::InsertIfNotPresent(&_viewIds, viewPairId.first, _viewIds.size());
    con::InsertIfNotPresent(&_viewIds, viewPairId.second, _viewIds.size());
    const int view1_index = _viewIds[viewPairId.first];
    const int view2_index = _viewIds[viewPairId.second];

    // Add the corresponding entries to A^t * A. Note that for each row, we may
    // represent the row as a block matrix: [B | C]. Thus, the corresponding
    // entries for A^t * A are:
    //
    //   [B | C]^t * [B | C] = [B^t * B | B^t * C]
    //                         [C^t * B | C^t * C]
    //
    // Note that for rotation matrices, B^t * B = C^t * B = I. Thus we have:
    //
    //   [B | C]^t * [B | C] = [   I    | B^t * C]
    //                         [C^t * B |    I   ]
    //
    // We only store the upper triangular portion of the matrix since it is
    // symmetric. In our case, one of B or C will be the identity, which further
    // simplifies the matrix entries.
    //
    // First, add the identity entries along the diagonal.
    for (int i = 0; i < 3; i++) {
        constraint_entries_.emplace_back(
            kNumRotationMatrixDimensions * view1_index + i,
            kNumRotationMatrixDimensions * view1_index + i, 1.0);
        constraint_entries_.emplace_back(
            kNumRotationMatrixDimensions * view2_index + i,
            kNumRotationMatrixDimensions * view2_index + i, 1.0);
    }

    // Add the 3x3 matrix B^t * C. This corresponds either to -R_ij or -R_ij^t
    // depending on the order of the view indices.
    Matrix3d relative_rotation_matrix;
    ceres::AngleAxisToRotationMatrix(
        relative_rotation.data(),
        ceres::ColumnMajorAdapter3x3(relative_rotation_matrix.data()));

    // We seek to insert B^t * C into the linear system as noted above. If the
    // view1 index comes before the view2 index, then B = -R_ij and C =
    // I. Otherwise, B = I and C = -R_ij.
    if (view1_index < view2_index) {
        Fill3x3SparseMatrix(-relative_rotation_matrix.transpose(),
                            kNumRotationMatrixDimensions * view1_index,
                            kNumRotationMatrixDimensions * view2_index,
                            &constraint_entries_);
    }
    else {
        Fill3x3SparseMatrix(-relative_rotation_matrix,
                            kNumRotationMatrixDimensions * view2_index,
                            kNumRotationMatrixDimensions * view1_index,
                            &constraint_entries_);
    }
}

// Given the relative rotation constraints added with
// AddRelativeRotationConstraint, this method returns the robust estimation of
// global camera orientations. Like the method above, this requires an initial
// estimate of the global orientations.
bool LinearRotationEstimator::EstimateRotations(
    std::unordered_map<ViewId, Eigen::Vector3d>* orientations)
{
    CHECK(!constraint_entries_.empty());
    CHECK_NOTNULL(orientations);

    // Setup the sparse linear system.
    SparseMatrixd constraint_matrix(
        _viewIds.size() * kNumRotationMatrixDimensions,
        _viewIds.size() * kNumRotationMatrixDimensions);
    constraint_matrix.setFromTriplets(constraint_entries_.begin(),
                                      constraint_entries_.end());

    // Compute the 3 eigenvectors corresponding to the smallest eigenvalues.
    // These orthogonal vectors will contain the solution rotation matrices.
    auto linear_solver = std::make_shared<SparseCholeskyLLt>();
    SparseSymShiftSolveLLT<double> op(linear_solver, constraint_matrix);
    Spectra::SymEigsShiftSolver<SparseSymShiftSolveLLT<double>> eigs(op, 3, 6,
                                                                     0.0);
    eigs.init();
    eigs.compute(Spectra::SortRule::LargestMagn, 1000, 1e-4);

    // The solution appears in the first three eigenvectors.
    const MatrixXd solution =
        eigs.eigenvectors().leftCols<kNumRotationMatrixDimensions>();

    // Project all solutions into a valid SO3 rotation space. The linear system
    // above makes no constraint on the space of the solutions, so the final
    // solutions are not guaranteed to be valid rotations (e.g., det(R) may not
    // be +1).
    orientations->reserve(_viewIds.size());
    for (const auto& [viewId, index] : _viewIds) {
        const Matrix3d non_so3_rotation =
            solution.block<kNumRotationMatrixDimensions,
                           kNumRotationMatrixDimensions>(
                kNumRotationMatrixDimensions * index, 0);
        const Matrix3d rotation = projectToRotationMatrix(non_so3_rotation);

        Vector3d rvec;
        ceres::RotationMatrixToAngleAxis(
            ceres::ColumnMajorAdapter3x3(rotation.data()), rvec.data());
        orientations->emplace(viewId, rvec);
    }

    return true;
}

} // namespace tl
