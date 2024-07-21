#include "ludpositionestimator.h"

#include <unordered_set>

#include <ceres/rotation.h>
#include <Eigen/SparseCore>

#include <tCore/ContainerUtils>

#include <tMath/Solvers/ConstrainedL1Solver>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {

constexpr int kConstantViewIndex = -3;

Vector3d GetRotatedTranslation(const Eigen::Vector3d& rvec,
                               const Eigen::Vector3d& translation)
{
    Matrix3d R;
    ceres::AngleAxisToRotationMatrix(rvec.data(),
                                     ceres::ColumnMajorAdapter3x3(R.data()));
    return R.transpose() * translation;
}

} // namespace

LeastUnsquaredDeviationPositionEstimator::
    LeastUnsquaredDeviationPositionEstimator(const Options& opts)
    : PositionEstimator(), _opts(opts)
{
    CHECK_GT(_opts.max_num_iterations, 0);
    CHECK_GT(_opts.max_num_reweighted_iterations, 0);
}

bool LeastUnsquaredDeviationPositionEstimator::EstimatePositions(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    CHECK_NOTNULL(positions)->clear();

    InitializeIndexMapping(viewPairs, orientations);

    const int num_views = _viewIdToIndex.size();
    const int num_view_pairs = _viewPairIdToIndex.size();

    // Set up the linear system.
    SetupConstraintMatrix(viewPairs, orientations);
    VectorXd solution;
    solution.setZero(constraint_matrix_.cols());

    // Create the lower bound constraint enforcing that all scales are > 1.
    Eigen::SparseMatrix<double> geq_mat(num_view_pairs,
                                        constraint_matrix_.cols());
    for (int i = 0; i < num_view_pairs; i++) {
        geq_mat.insert(i, 3 * (num_views - 1) + i) = 1.0;
    }

    VectorXd geq_vec(num_view_pairs);
    geq_vec.setConstant(1.);

    VectorXd b(constraint_matrix_.rows());
    b.setZero();

    // Solve for camera positions by solving a constrained L1 problem to enforce
    // all relative translations scales > 1.
    ConstrainedL1Solver::Options l1_options;
    ConstrainedL1Solver solver(l1_options, constraint_matrix_, b, geq_mat,
                               geq_vec);
    solver.Solve(&solution);

    // Set the estimated positions.
    for (const auto& [viewId, index] : _viewIdToIndex) {
        if (index == kConstantViewIndex) {
            (*positions)[viewId] = Vector3d::Zero();
        }
        else {
            (*positions)[viewId] = solution.segment<3>(index);
        }
    }

    return true;
}

void LeastUnsquaredDeviationPositionEstimator::InitializeIndexMapping(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations)
{
    std::unordered_set<ViewId> viewIds;
    for (const auto& [viewPairId, _] : viewPairs) {
        const auto& viewId1 = viewPairId.first;
        const auto& viewId2 = viewPairId.second;

        if (orientations.contains(viewId1) && orientations.contains(viewId2)) {
            viewIds.insert(viewId1);
            viewIds.insert(viewId2);
        }
    }

    // Create a mapping from the view id to the index of the linear system.
    int index = kConstantViewIndex;
    _viewIdToIndex.reserve(viewIds.size());
    for (const auto& viewId : viewIds) {
        _viewIdToIndex[viewId] = index;
        index += 3;
    }

    // Create a mapping from the view id pair to the index of the linear system.
    _viewPairIdToIndex.reserve(viewPairs.size());
    for (const auto& [viewPairId, _] : viewPairs) {
        if (_viewIdToIndex.contains(viewPairId.first) &&
            _viewIdToIndex.contains(viewPairId.second)) {
            _viewPairIdToIndex[viewPairId] = index;
            ++index;
        }
    }
}

void LeastUnsquaredDeviationPositionEstimator::SetupConstraintMatrix(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations)
{
    constraint_matrix_.resize(
        3 * _viewPairIdToIndex.size(),
        3 * (_viewIdToIndex.size() - 1) + viewPairs.size());

    // Add the camera to camera constraints.
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(9 * viewPairs.size());
    int row = 0;
    for (const auto& [viewPairId, viewPairInfo] : viewPairs) {
        const auto& viewId1 = viewPairId.first;
        const auto& viewId2 = viewPairId.second;

        if (!_viewIdToIndex.contains(viewId1) ||
            !_viewIdToIndex.contains(viewId2)) {
            continue;
        }

        const auto view1Index = con::FindOrDie(_viewIdToIndex, viewId1);
        const auto view2Index = con::FindOrDie(_viewIdToIndex, viewId2);
        const auto scaleIndex =
            con::FindOrDieNoPrint(_viewPairIdToIndex, viewPairId);

        // Rotate the relative translation so that it is aligned to the global
        // orientation frame.
        const Vector3d translation_direction = GetRotatedTranslation(
            con::FindOrDie(orientations, viewId1), viewPairInfo.position);

        // Add the constraint for view 1 in the minimization:
        //   position2 - position1 - scale_1_2 * translation_direction.
        if (view1Index != kConstantViewIndex) {
            triplets.emplace_back(row + 0, view1Index + 0, -1.);
            triplets.emplace_back(row + 1, view1Index + 1, -1.);
            triplets.emplace_back(row + 2, view1Index + 2, -1.);
        }

        // Add the constraint for view 2 in the minimization:
        //   position2 - position1 - scale_1_2 * translation_direction.
        if (view2Index != kConstantViewIndex) {
            triplets.emplace_back(row + 0, view2Index + 0, 1.);
            triplets.emplace_back(row + 1, view2Index + 1, 1.);
            triplets.emplace_back(row + 2, view2Index + 2, 1.);
        }

        // Add the constraint for scale in the minimization:
        //   position2 - position1 - scale_1_2 * translation_direction.
        triplets.emplace_back(row + 0, scaleIndex, -translation_direction[0]);
        triplets.emplace_back(row + 1, scaleIndex, -translation_direction[1]);
        triplets.emplace_back(row + 2, scaleIndex, -translation_direction[2]);

        row += 3;
    }

    constraint_matrix_.setFromTriplets(triplets.begin(), triplets.end());

    VLOG(2) << viewPairs.size()
            << " camera to camera constraints were added "
               "to the position estimation problem.";
}

} // namespace tl
