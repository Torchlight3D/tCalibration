#pragma once

#include <Eigen/SparseCore>

#include "positionestimator.h"

namespace tl {

struct ViewPairInfo;

// Brief:
// Estimates the camera position of views given pairwise relative poses and the
// absolute orientations of cameras. Positions are estimated using a least
// unsquared deviations solver -- essentially an L1 solver that is wrapped in an
// Iteratively Reweighted Least Squares (IRLS) formulation.
//
// Ref:
// "Robust Camera Location Estimation by Convex Programming" by Ozyesil and
// Singer (CVPR 2015)
class LeastUnsquaredDeviationPositionEstimator final : public PositionEstimator
{
public:
    struct Options
    {
        // Options for ADMM QP solver.
        int max_num_iterations = 400;

        // Maximum number of reweighted iterations.
        int max_num_reweighted_iterations = 10;

        // A measurement for convergence criterion.
        double convergence_criterion = 1e-4;
    };

    explicit LeastUnsquaredDeviationPositionEstimator(const Options& options);

    bool EstimatePositions(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientation,
        std::unordered_map<ViewId, Eigen::Vector3d>* positions) override;

private:
    void InitializeIndexMapping(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations);

    // Creates camera to camera constraints from relative translations.
    void SetupConstraintMatrix(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        const std::unordered_map<ViewId, Eigen::Vector3d>& orientations);

private:
    const Options _opts;

    std::unordered_map<ViewIdPair, int> _viewPairIdToIndex;
    std::unordered_map<ViewId, int> _viewIdToIndex;

    Eigen::SparseMatrix<double> constraint_matrix_;

    // friend class EstimatePositionsLeastUnsquaredDeviationTest;

    // DISALLOW_COPY_AND_ASSIGN(LeastUnsquaredDeviationPositionEstimator);
};

} // namespace tl
