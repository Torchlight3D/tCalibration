#include "estimaterotationutils.h"

#include <tCore/ContainerUtils>
#include <tMvs/ViewPairInfo>

namespace tl {

using Tripletd = Eigen::Triplet<double>;

void ViewIdToAscentIndex(
    const std::unordered_map<ViewId, Eigen::Vector3d>& rotations,
    std::unordered_map<ViewId, int>* viewIdToIndex)
{
    std::vector<ViewId> viewIds;
    for (const auto& [viewId, _] : rotations) {
        viewIds.push_back(viewId);
    }

    std::sort(viewIds.begin(), viewIds.end());

    for (size_t i = 0; i < viewIds.size(); i++) {
        (*viewIdToIndex)[viewIds[i]] = i;
    }
}

void SetupLinearSystem(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& relative_rotations,
    size_t num_rotations, const std::unordered_map<ViewId, int>& viewIdToIndex,
    Eigen::SparseMatrix<double>* sparse_matrix)
{
    // The rotation change is one less than the number of global rotations
    // because we keep one rotation constant.
    (*sparse_matrix)
        .resize(relative_rotations.size() * 3, (num_rotations - 1) * 3);

    constexpr int kStartRotationIndex = -1;

    // For each relative rotation constraint, add an entry to the sparse
    // matrix. We use the first order approximation of angle axis such that:
    // R_ij = R_j - R_i. This makes the sparse matrix just a bunch of identity
    // matrices.
    int rotation_error_index = 0;
    std::vector<Tripletd> triplets;
    for (const auto& [pairId, _] : relative_rotations) {
        const int view1_index = con::FindOrDie(viewIdToIndex, pairId.first) - 1;
        if (view1_index != kStartRotationIndex) {
            triplets.emplace_back(3 * rotation_error_index + 0,
                                  3 * view1_index + 0, -1.);
            triplets.emplace_back(3 * rotation_error_index + 1,
                                  3 * view1_index + 1, -1.);
            triplets.emplace_back(3 * rotation_error_index + 2,
                                  3 * view1_index + 2, -1.);
        }

        const int view2_index =
            con::FindOrDie(viewIdToIndex, pairId.second) - 1;
        if (view2_index != kStartRotationIndex) {
            triplets.emplace_back(3 * rotation_error_index + 0,
                                  3 * view2_index + 0, 1.);
            triplets.emplace_back(3 * rotation_error_index + 1,
                                  3 * view2_index + 1, 1.);
            triplets.emplace_back(3 * rotation_error_index + 2,
                                  3 * view2_index + 2, 1.);
        }

        ++rotation_error_index;
    }

    sparse_matrix->setFromTriplets(triplets.begin(), triplets.end());
}

} // namespace tl
