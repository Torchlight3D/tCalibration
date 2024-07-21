#include "nonlinearrotationestimator.h"

#include <memory>
#include <unordered_map>

#include <Eigen/Core>
#include <ceres/ceres.h>

#include <tCore/ContainerUtils>

namespace tl {

using Eigen::Vector3d;

ceres::CostFunction* PairwiseRotationError::create(
    const Eigen::Vector3d& relative_rotation, double weight)
{
    return new ceres::AutoDiffCostFunction<
        PairwiseRotationError, Vector3d::SizeAtCompileTime,
        Vector3d::SizeAtCompileTime, Vector3d::SizeAtCompileTime>(
        new PairwiseRotationError(relative_rotation, weight));
}

NonlinearRotationEstimator::NonlinearRotationEstimator(double robust_loss_width)
    : robust_loss_width_(robust_loss_width)
{
}

bool NonlinearRotationEstimator::EstimateRotations(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* global_orientations)
{
    CHECK_NOTNULL(global_orientations);
    if (global_orientations->empty()) {
        LOG(INFO) << "Skipping nonlinear rotation optimization because no "
                     "initialization was provivded.";
        return false;
    }
    if (viewPairs.empty()) {
        LOG(INFO) << "Skipping nonlinear rotation optimization because no "
                     "relative rotation constraints were provivded.";
        return false;
    }

    // Set up the problem and loss function.
    std::unique_ptr<ceres::Problem> problem(new ceres::Problem());
    auto* loss = new ceres::SoftLOneLoss(robust_loss_width_);

    for (const auto& [pair, info] : viewPairs) {
        Vector3d* rotation1 = con::FindOrNull(*global_orientations, pair.first);
        Vector3d* rotation2 =
            con::FindOrNull(*global_orientations, pair.second);

        // Do not add the relative rotation constaint if it requires an
        // orientation that we do not have an initialization for.
        if (!rotation1 || !rotation2) {
            continue;
        }

        auto cost = PairwiseRotationError::create(info.rotation, 1.);
        problem->AddResidualBlock(cost, loss, rotation1->data(),
                                  rotation2->data());
    }

    // The problem should be relatively sparse so sparse cholesky is a good
    // choice.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 200;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem.get(), &summary);

    VLOG(1) << summary.FullReport();

    return true;
}

} // namespace tl
