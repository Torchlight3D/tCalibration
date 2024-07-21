#pragma once

#include <unordered_map>

#include <Eigen/SparseCore>

#include <tMvs/Types>

namespace tl {

struct ViewPairInfo;

// The id of rotations is re-indexed for convenience of matrix manipulation.
void ViewIdToAscentIndex(
    const std::unordered_map<ViewId, Eigen::Vector3d>& rotations,
    std::unordered_map<ViewId, int>* viewIdToIndex);

// Sets up the sparse linear system such that dR_ij = dR_j - dR_i. This is the
// first-order approximation of the angle-axis rotations. This should only be
// called once.
void SetupLinearSystem(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& relative_rotations,
    size_t num_rotations, const std::unordered_map<ViewId, int>& viewIdToIndex,
    Eigen::SparseMatrix<double>* sparse_matrix);

} // namespace tl
