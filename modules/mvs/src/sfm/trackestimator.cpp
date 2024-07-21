#include "trackestimator.h"

#include <memory>

#include <Eigen/Core>
#include <glog/logging.h>

#include "../desc/scene.h"
#include "threadpool.h"

namespace tl {

using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

void GetObservationsFromTrackViews(TrackId trackId, Scene& scene,
                                   std::vector<ViewId>* viewIds,
                                   std::vector<Eigen::Vector2d>* features,
                                   std::vector<Eigen::Vector3d>* origins,
                                   std::vector<Matrix34d>* proj_matrices,
                                   std::vector<Eigen::Vector3d>* ray_directions)
{
    auto* track = scene.rTrack(trackId);
    for (const auto& viewId : track->viewIds()) {
        const auto* view = scene.view(viewId);
        if (!view || !view->estimated()) {
            continue;
        }

        // If the feature is not in the view then we have an ill-formed
        // reconstruction.
        const auto* feature = CHECK_NOTNULL(view->featureOf(trackId));
        const Vector3d ray_dir =
            view->camera().pixelToUnitDepthRay(feature->pos).normalized();

        features->emplace_back(feature->pos);
        viewIds->emplace_back(viewId);
        origins->emplace_back(view->camera().position());

        Matrix34d proj_mat;
        view->camera().projectionMatrix(proj_mat);
        proj_matrices->emplace_back(proj_mat);
        ray_directions->emplace_back(ray_dir);
    }
}

// Returns false if the reprojection error of the triangulated point is greater
// than the max allowable reprojection error (for any observation) and true
// otherwise.
bool AcceptableReprojectionError(const Scene& scene, TrackId trackId,
                                 const std::vector<ViewId>& viewIds,
                                 const std::vector<Eigen::Vector2d>& features,
                                 double sq_max_reprojection_error_pixels)
{
    const Track& track = *scene.track(trackId);
    int num_projections = 0;
    double mean_sq_reprojection_error = 0;
    for (size_t i = 0; i < viewIds.size(); i++) {
        // We do not need to check if the view is present or estimated, since
        // this was done prior to the input.
        const auto* view = scene.view(viewIds[i]);
        const auto& camera = view->camera();
        Vector2d reprojection;
        if (camera.projectPoint(track.position(), reprojection) < 0.) {
            return false;
        }

        mean_sq_reprojection_error +=
            (features[i] - reprojection).squaredNorm();
        ++num_projections;
    }

    return (mean_sq_reprojection_error / static_cast<double>(num_projections)) <
           sq_max_reprojection_error_pixels;
}

} // namespace

TrackEstimator::TrackEstimator(const Options& options, Scene* scene)
    : options_(options), _scene(scene)
{
}

TrackEstimator::Summary TrackEstimator::EstimateAllTracks()
{
    std::unordered_set<TrackId> trackIds;
    for (const auto& viewId : _scene->viewIds()) {
        if (const auto* view = _scene->view(viewId);
            view && view->estimated()) {
            const auto trackIdsInView = view->trackIds();
            trackIds.insert(trackIdsInView.cbegin(), trackIdsInView.cend());
        }
    }

    return EstimateTracks(trackIds);
}

TrackEstimator::Summary TrackEstimator::EstimateTracks(
    const std::unordered_set<TrackId>& trackIds)
{
    // Get all unestimated track ids.
    tracks_to_estimate_.clear();
    tracks_to_estimate_.reserve(trackIds.size());
    for (const auto& trackId : trackIds) {
        if (const auto* track = _scene->track(trackId); !track->estimated()) {
            tracks_to_estimate_.emplace_back(trackId);
        }
    }

    summary_ = {};
    summary_.input_num_estimated_tracks =
        trackIds.size() - tracks_to_estimate_.size();
    summary_.num_triangulation_attempts = tracks_to_estimate_.size();

    // Exit early if there are no tracks to estimate.
    if (tracks_to_estimate_.empty()) {
        return summary_;
    }

    num_bad_angles_ = 0;
    num_failed_triangulations_ = 0;
    num_bad_reprojections_ = 0;

    // Estimate the tracks in parallel. Instead of 1 threadpool worker per
    // track, we let each worker estimate a fixed number of tracks at a time
    // (e.g. 20 tracks). Since estimating the tracks is so fast, this strategy
    // is better helps speed up multithreaded estimation by reducing the
    // overhead of starting/stopping threads.
    const int num_threads = std::min(
        options_.num_threads, static_cast<int>(tracks_to_estimate_.size()));
    const int interval_step =
        std::min(options_.multithreaded_step_size,
                 static_cast<int>(tracks_to_estimate_.size()) / num_threads);

    std::unique_ptr<ThreadPool> pool(new ThreadPool(num_threads));
    for (int i = 0; i < tracks_to_estimate_.size(); i += interval_step) {
        const int end_interval = std::min(
            static_cast<int>(tracks_to_estimate_.size()), i + interval_step);
        pool->Add(&TrackEstimator::EstimateTrackSet, this, i, end_interval);
    }
    pool.reset(nullptr);

    LOG(INFO) << summary_.estimated_tracks.size()
              << " tracks were estimated of "
              << summary_.num_triangulation_attempts << " possible tracks. "
              << num_bad_angles_
              << " triangulations failed due to bad triangulation angles and "
              << num_bad_reprojections_
              << " triangulations failed with too high reprojection errors.";
    return summary_;
}

void TrackEstimator::EstimateTrackSet(TrackId start, TrackId end)
{
    std::unordered_set<TrackId> estimatedTrackIds;
    for (int i = start; i < end; i++) {
        if (EstimateTrack(tracks_to_estimate_[i])) {
            estimatedTrackIds.emplace(tracks_to_estimate_[i]);
        }
    }

    // Add the estimated tracks to the output summary.
    {
        std::lock_guard<std::mutex> guard{summary_mutex_};
        summary_.estimated_tracks.insert(estimatedTrackIds.begin(),
                                         estimatedTrackIds.end());
    }
}

bool TrackEstimator::EstimateTrack(TrackId trackId)
{
    constexpr int kMinNumObservationsForTriangulation = 2;

    auto* track = _scene->rTrack(trackId);
    if (track->estimated()) {
        return true;
    }

    // Gather projection matrices and features.
    std::vector<ViewId> viewIds;
    std::vector<Vector2d> features;
    std::vector<Vector3d> origins;
    std::vector<Vector3d> ray_directions;
    std::vector<Matrix34d> norm_proj_matrices;
    GetObservationsFromTrackViews(trackId, *_scene, &viewIds, &features,
                                  &origins, &norm_proj_matrices,
                                  &ray_directions);

    // Check the angle between views.
    if (viewIds.size() < kMinNumObservationsForTriangulation ||
        !SufficientTriangulationAngle(
            ray_directions, options_.min_triangulation_angle_degrees)) {
        ++num_bad_angles_;
        return false;
    }

    // Triangulate the track
    bool triangulated{false};
    switch (options_.triangulation_method) {
        case TriangulationMethodType::SVD:
            triangulated = TriangulateNViewSVD(norm_proj_matrices, features,
                                               &track->rPosition());
            break;
        case TriangulationMethodType::MIDPOINT:
            triangulated = TriangulateMidpoint(origins, ray_directions,
                                               &track->rPosition());
            break;
        case TriangulationMethodType::L2_MINIMIZATION:
            triangulated = TriangulateNView(norm_proj_matrices, features,
                                            &track->rPosition());
            break;
        default:
            triangulated = TriangulateMidpoint(origins, ray_directions,
                                               &track->rPosition());
            break;
    }

    if (!triangulated) {
        ++num_failed_triangulations_;
        return false;
    }

    // Set the inverse depth of the track
    if (options_.ba_options.use_inverse_depth_parametrization) {
        const auto refViewId = track->referenceViewId();
        if (refViewId == kInvalidViewId) {
            track->setEstimated(false);
            return false;
        }

        const auto* refView = _scene->view(refViewId);
        if (!refView || !refView->estimated()) {
            track->setEstimated(false);
            return false;
        }

        const auto& refCam = refView->camera();
        Vector2d point_i;
        const auto depth = refCam.projectPoint(track->position(), point_i);
        track->setInverseDepth(1. / depth);
        track->setReferenceBearing(refCam.pixelToNormalizedCoordinates(
            refView->featureOf(trackId)->pos));
    }

    // Bundle adjust the track.
    if (options_.bundle_adjustment) {
        track->setEstimated(true);
        const auto summary =
            BundleAdjustTrack(options_.ba_options, trackId, _scene);
        track->setEstimated(false);
        if (options_.ba_options.use_inverse_depth_parametrization) {
            if (*track->inverseDepth() <= 0.) {
                track->setEstimated(false);
                return false;
            }
        }

        if (!summary.success) {
            return false;
        }
    }

    // Ensure the reprojection errors are acceptable.
    const double sq_max_reprojection_error_pixels =
        options_.max_acceptable_reprojection_error_pixels *
        options_.max_acceptable_reprojection_error_pixels;

    if (!AcceptableReprojectionError(*_scene, trackId, viewIds, features,
                                     sq_max_reprojection_error_pixels)) {
        ++num_bad_reprojections_;
        return false;
    }

    track->setEstimated(true);

    return true;
}

} // namespace tl
