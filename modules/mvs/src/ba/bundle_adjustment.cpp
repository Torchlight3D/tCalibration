#include "bundle_adjustment.h"
#include "bundle_adjustment_solver.h"

#include <tMvs/Scene>

namespace tl {

BundleAdjustmentSummary BundleAdjustScene(
    const BundleAdjustmentOptions& options, Scene* scene)
{
    CHECK_NOTNULL(scene);

    BundleAdjuster ba{options, scene};
    for (const auto& viewId : scene->viewIds()) {
        ba.addView(viewId);
    }
    for (const auto& trackId : scene->trackIds()) {
        ba.addTrack(trackId);
    }

    return ba.optimize();
}

BundleAdjustmentSummary BundleAdjustPartialScene(
    const BundleAdjustmentOptions& options,
    const std::unordered_set<ViewId>& viewIds,
    const std::unordered_set<TrackId>& trackIds, Scene* scene)
{
    CHECK_NOTNULL(scene);

    // TODO: Check contains or not before adding?
    BundleAdjuster ba{options, scene};
    for (const auto& viewId : viewIds) {
        ba.addView(viewId);
    }
    for (const auto& trackId : trackIds) {
        ba.addTrack(trackId);
    }

    return ba.optimize();
}

BundleAdjustmentSummary BundleAdjustPartialViewsConstant(
    const BundleAdjustmentOptions& options,
    const std::vector<ViewId>& mutableViewIds,
    const std::vector<ViewId>& constViewIds, Scene* scene)
{
    CHECK_NOTNULL(scene);

    BundleAdjuster ba{options, scene};
    for (const auto& viewId : mutableViewIds) {
        ba.addView(viewId);
    }
    for (const auto& viewId : constViewIds) {
        ba.addView(viewId);
        ba.setCameraExtrinsicsConstant(viewId);
    }
    for (const auto& trackId : scene->trackIds()) {
        ba.addTrack(trackId);
    }

    return ba.optimize();
}

BundleAdjustmentSummary BundleAdjustView(const BundleAdjustmentOptions& options,
                                         ViewId viewId, Scene* scene)
{
    BundleAdjustmentOptions opts = options;
    opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.use_inner_iterations = false;

    BundleAdjuster ba{opts, scene};
    ba.addView(viewId);

    return ba.optimize();
}

BundleAdjustmentSummary BundleAdjustView(const BundleAdjustmentOptions& options,
                                         ViewId viewId, Scene* scene,
                                         Matrix6d* covariance, double* variance)
{
    BundleAdjustmentOptions opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjuster ba{opts, scene};
    ba.addView(viewId);

    auto summary = ba.optimize();
    if (!summary.success) {
        *covariance = Matrix6d::Identity();
        return summary;
    }

    if (!ba.calcCovarianceForView(viewId, covariance)) {
        summary.success = false;
        *variance = 1.0;
    }
    else {
        const double redundancy = scene->view(viewId)->featureCount() * 2. - 6.;
        *variance = (2. * summary.final_cost) / redundancy;
        *covariance *= (*variance);
    }

    return summary;
}

BundleAdjustmentSummary BundleAdjustViews(
    const BundleAdjustmentOptions& options, const std::vector<ViewId>& viewIds,
    Scene* scene)
{
    BundleAdjustmentOptions opts = options;
    opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.use_inner_iterations = false;

    BundleAdjuster ba{opts, scene};
    for (const auto& viewId : viewIds) {
        ba.addView(viewId);
    }

    return ba.optimize();
}

BundleAdjustmentSummary BundleAdjustViews(
    const BundleAdjustmentOptions& options, const std::vector<ViewId>& viewIds,
    Scene* scene, std::map<ViewId, Matrix6d>& covariances, double* variance)
{
    BundleAdjustmentOptions opts = options;
    opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.use_inner_iterations = false;

    BundleAdjuster ba{opts, scene};
    for (const auto& viewId : viewIds) {
        ba.addView(viewId);
    }

    auto ba_summary = ba.optimize();
    if (!ba_summary.success) {
        return ba_summary;
    }

    if (!ba.calcCovarianceForViews(viewIds, covariances)) {
        ba_summary.success = false;
        *variance = 1.;
    }
    else {
        int observationCount{0};
        for (const auto& viewId : viewIds) {
            observationCount += scene->view(viewId)->featureCount();
        }

        const int totalVariableCount = viewIds.size() * 6;
        const double redundancy = observationCount * 2. - totalVariableCount;

        *variance = (2.0 * ba_summary.final_cost) / redundancy;
        for (auto& [_, cov] : covariances) {
            cov *= (*variance);
        }
    }

    return ba_summary;
}

BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options, TrackId trackId, Scene* scene)
{
    BundleAdjustmentOptions opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjuster ba{opts, scene};
    ba.addTrack(trackId);
    return ba.optimize();
}

BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options, TrackId trackId, Scene* scene,
    Eigen::Matrix3d* covariance, double* variance)
{
    BundleAdjustmentOptions opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjuster ba{opts, scene};
    // Use homo representation, otherwise covariance matrix will be singular
    ba.addTrack(trackId);

    auto ba_summary = ba.optimize();
    if (!ba_summary.success) {
        *covariance = Eigen::Matrix3d::Identity();
        return ba_summary;
    }

    if (!ba.calcCovarianceForTrack(trackId, covariance)) {
        ba_summary.success = false;
        *variance = 1.;
    }
    else {
        const double redundancy = scene->rTrack(trackId)->viewCount() * 2. - 3.;
        *variance = (2. * ba_summary.final_cost) / redundancy;
        *covariance *= (*variance);
    }

    return ba_summary;
}

BundleAdjustmentSummary BundleAdjustTracks(
    const BundleAdjustmentOptions& options,
    const std::vector<TrackId>& trackIds, Scene* scene)
{
    BundleAdjustmentOptions opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjuster ba{opts, scene};
    for (const auto& trackId : trackIds) {
        ba.addTrack(trackId);
    }

    return ba.optimize();
}

BundleAdjustmentSummary BundleAdjustTracks(
    const BundleAdjustmentOptions& options,
    const std::vector<TrackId>& trackIds, Scene* scene,
    std::map<TrackId, Eigen::Matrix3d>* covariances, double* variance)
{
    BundleAdjustmentOptions ba_options = options;
    ba_options.linear_solver_type = ceres::DENSE_QR;
    ba_options.use_inner_iterations = false;

    BundleAdjuster ba{ba_options, scene};
    for (const auto& trackId : trackIds) {
        // Use homo representation, otherwise covariance matrix will be singular
        ba.addTrack(trackId);
    }

    auto ba_summary = ba.optimize();
    if (!ba_summary.success) {
        return ba_summary;
    }

    if (!ba.calcCovarianceForTracks(trackIds, covariances)) {
        ba_summary.success = false;
        *variance = 1.0;
    }
    else {
        int observationCount{0};
        for (const auto& trackId : trackIds) {
            observationCount += scene->track(trackId)->viewCount();
        }

        const auto totalVariableCount = trackIds.size() * 3;
        const double redundancy = observationCount * 2. - totalVariableCount;

        *variance = (2.0 * ba_summary.final_cost) / redundancy;
        for (auto& [_, cov] : *covariances) {
            cov *= (*variance);
        }
    }

    return ba_summary;
}

} // namespace tl
