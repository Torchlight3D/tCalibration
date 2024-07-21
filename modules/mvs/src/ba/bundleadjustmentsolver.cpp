#include "bundleadjustment.h"

#include <glog/logging.h>

namespace tl {

using Eigen::Matrix3d;

BundleAdjustment::Summary BundleAdjustScene(
    const BundleAdjustment::Options& options, Scene* scene)
{
    CHECK_NOTNULL(scene);

    BundleAdjustment ba{options, scene};
    for (const auto& viewId : scene->viewIds()) {
        ba.addView(viewId);
    }
    for (const auto& trackId : scene->trackIds()) {
        ba.addTrack(trackId);
    }

    return ba.optimize();
}

BundleAdjustment::Summary BundleAdjustPartialScene(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& viewIds, const std::vector<TrackId>& trackIds,
    Scene* scene)
{
    CHECK_NOTNULL(scene);

    // TODO: Check contains or not before adding?
    BundleAdjustment ba{options, scene};
    for (const auto& viewId : viewIds) {
        ba.addView(viewId);
    }
    for (const auto& trackId : trackIds) {
        ba.addTrack(trackId);
    }

    return ba.optimize();
}

BundleAdjustment::Summary BundleAdjustPartialViewsConstant(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& mutableViewIds,
    const std::vector<ViewId>& constViewIds, Scene* scene)
{
    CHECK_NOTNULL(scene);

    BundleAdjustment ba{options, scene};
    for (const auto& viewId : mutableViewIds) {
        ba.addView(viewId);
    }
    for (const auto& viewId : constViewIds) {
        ba.addView(viewId);
        ba.setFixedView(viewId);
    }
    for (const auto& trackId : scene->trackIds()) {
        ba.addTrack(trackId);
    }

    return ba.optimize();
}

BundleAdjustment::Summary BundleAdjustView(
    const BundleAdjustment::Options& options, ViewId viewId, Scene* scene)
{
    auto opts = options;
    opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
    ba.addView(viewId);

    return ba.optimize();
}

BundleAdjustment::Summary BundleAdjustView(
    const BundleAdjustment::Options& options, ViewId viewId, Scene* scene,
    Matrix6d* covariance, double* variance)
{
    auto opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
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
        const double redundancy = scene->view(viewId)->featureCount() * 2. - 6;
        *variance = (2. * summary.final_cost) / redundancy;
        *covariance *= (*variance);
    }

    return summary;
}

BundleAdjustment::Summary BundleAdjustViews(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& viewIds, Scene* scene)
{
    auto opts = options;
    opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
    for (const auto& viewId : viewIds) {
        ba.addView(viewId);
    }

    return ba.optimize();
}

BundleAdjustment::Summary BundleAdjustViews(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& viewIds, Scene* scene,
    std::map<ViewId, Matrix6d>* covariances, double* variance)
{
    auto opts = options;
    opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
    for (const auto& viewId : viewIds) {
        ba.addView(viewId);
    }

    auto summary = ba.optimize();
    if (!summary.success) {
        return summary;
    }

    if (!ba.calcCovarianceForViews(viewIds, covariances)) {
        summary.success = false;
        *variance = 1.;
    }
    else {
        int observationCount{0};
        for (const auto& viewId : viewIds) {
            observationCount += scene->view(viewId)->featureCount();
        }

        const int totalVariableCount = viewIds.size() * 6;
        const double redundancy = observationCount * 2. - totalVariableCount;

        *variance = (2. * summary.final_cost) / redundancy;
        for (auto& [_, cov] : *covariances) {
            cov *= (*variance);
        }
    }

    return summary;
}

BundleAdjustment::Summary BundleAdjustViews(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& viewIds, Scene* scene, CameraId camId,
    Eigen::MatrixXd* covariance, double* variance)
{
    auto opts = options;
    opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
    for (const auto& viewId : viewIds) {
        ba.addView(viewId);
    }

    auto summary = ba.optimize();
    if (!summary.success) {
        return summary;
    }

    if (!ba.calcCovarianceForCamera(camId, covariance)) {
        summary.success = false;
        *variance = 1.;
    }
    else {
        // TODO: Double check here
        const double redundancy = scene->viewCount(camId) * 2. -
                                  scene->camera(camId)->parameters().size();

        *variance = (2. * summary.final_cost) / redundancy;
        *covariance *= (*variance);
    }

    return summary;
}

BundleAdjustment::Summary BundleAdjustTrack(
    const BundleAdjustment::Options& options, TrackId trackId, Scene* scene)
{
    auto opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
    ba.addTrack(trackId);
    return ba.optimize();
}

BundleAdjustment::Summary BundleAdjustTrack(
    const BundleAdjustment::Options& options, TrackId trackId, Scene* scene,
    Eigen::Matrix3d* covariance, double* variance)
{
    auto opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
    // Use homo representation, otherwise covariance matrix will be singular
    ba.addTrack(trackId);

    auto summary = ba.optimize();
    if (!summary.success) {
        *covariance = Matrix3d::Identity();
        return summary;
    }

    if (!ba.calcCovarianceForTrack(trackId, covariance)) {
        summary.success = false;
        *variance = 1.;
    }
    else {
        const double redundancy = scene->track(trackId)->viewCount() * 2. - 3;
        *variance = (2. * summary.final_cost) / redundancy;
        *covariance *= (*variance);
    }

    return summary;
}

BundleAdjustment::Summary BundleAdjustTracks(
    const BundleAdjustment::Options& options,
    const std::vector<TrackId>& trackIds, Scene* scene)
{
    auto opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
    for (const auto& trackId : trackIds) {
        ba.addTrack(trackId);
    }

    return ba.optimize();
}

BundleAdjustment::Summary BundleAdjustTracks(
    const BundleAdjustment::Options& options,
    const std::vector<TrackId>& trackIds, Scene* scene,
    std::map<TrackId, Eigen::Matrix3d>* covariances, double* variance)
{
    auto opts = options;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.use_inner_iterations = false;

    BundleAdjustment ba{opts, scene};
    for (const auto& trackId : trackIds) {
        // Use homo representation, otherwise covariance matrix will be singular
        ba.addTrack(trackId);
    }

    auto summary = ba.optimize();
    if (!summary.success) {
        return summary;
    }

    if (!ba.calcCovarianceForTracks(trackIds, covariances)) {
        summary.success = false;
        *variance = 1.0;
    }
    else {
        int observationCount{0};
        for (const auto& trackId : trackIds) {
            observationCount += scene->track(trackId)->viewCount();
        }

        const auto totalVariableCount = trackIds.size() * 3;
        const double redundancy = observationCount * 2. - totalVariableCount;

        *variance = (2. * summary.final_cost) / redundancy;
        for (auto& [_, cov] : *covariances) {
            cov *= (*variance);
        }
    }

    return summary;
}

} // namespace tl
