#include "nonlinearpositionestimator.h"

#include <ceres/rotation.h>

#include <tCamera/Camera>
#include <tCore/ContainerUtils>
#include <tCore/RandomGenerator>
#include <tMvs/Feature>
#include <tMvs/Scene>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {

Eigen::Vector3d GetRotatedTranslation(const Eigen::Vector3d& rvec,
                                      const Eigen::Vector3d& translation)
{
    Matrix3d R;
    ceres::AngleAxisToRotationMatrix(rvec.data(),
                                     ceres::ColumnMajorAdapter3x3(R.data()));
    return R.transpose() * translation;
}

Eigen::Vector3d GetRotatedFeatureRay(const Camera& camera,
                                     const Eigen::Vector3d& orientation,
                                     const Feature& feature)
{
    Camera temp_camera = camera;
    temp_camera.setOrientationFromAngleAxis(orientation);
    // Get the image ray rotated into the world reference frame.
    return camera.pixelToUnitDepthRay(feature.pos).normalized();
}

} // namespace

ceres::CostFunction* PairwiseTranslationError::create(
    const Eigen::Vector3d& translation_direction, double weight,
    double scale_estimate)
{
    return new ceres::AutoDiffCostFunction<
        PairwiseTranslationError, Vector3d::SizeAtCompileTime,
        Vector3d::SizeAtCompileTime, Vector3d::SizeAtCompileTime>(
        new PairwiseTranslationError(translation_direction, weight,
                                     scale_estimate));
}

NonlinearPositionEstimator::NonlinearPositionEstimator(const Options& options,
                                                       const Scene& scene)
    : options_(options), _scene(scene)
{
    CHECK_GT(options_.num_threads, 0);
    CHECK_GE(options_.min_num_points_per_view, 0);
    CHECK_GT(options_.point_to_camera_weight, 0);
    CHECK_GT(options_.robust_loss_width, 0);

    if (!options_.rng.get()) {
        rng_ = std::make_shared<RandomNumberGenerator>();
    }
    else {
        rng_ = options_.rng;
    }
}

bool NonlinearPositionEstimator::EstimatePositions(
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    CHECK_NOTNULL(positions);
    if (viewPairs.empty() || orientations.empty()) {
        VLOG(2) << "Failed to estimate global positions: "
                   "No view pairs and estimated orientations available";
        return false;
    }

    triangulated_points_.clear();
    problem_.reset(new ceres::Problem());
    _viewPairs = &viewPairs;

    // Iterative schur is only used if the problem is large enough, otherwise
    // sparse schur is used.
    constexpr int kMinNumCamerasForIterativeSolve = 1000;

    // Initialize positions to be random.
    if (_fixedViewIds.empty()) {
        InitializeRandomPositions(orientations, positions);
    }

    // Add the constraints to the problem.
    AddCameraToCameraConstraints(orientations, positions);
    if (options_.min_num_points_per_view > 0) {
        AddPointToCameraConstraints(orientations, positions);
        AddCamerasAndPointsToParameterGroups(positions);
    }

    // If the user did not specify fixed cams set one camera to be at the
    // origin to remove the ambiguity of the origin.
    if (_fixedViewIds.empty()) {
        positions->begin()->second.setZero();
        _fixedViewIds.insert(positions->begin()->first);
    }
    else {
        for (const auto& viewId : _fixedViewIds) {
            problem_->SetParameterBlockConstant(
                con::FindOrDie(*positions, viewId).data());
        }
    }

    solver_options_.num_threads = options_.num_threads;
    solver_options_.max_num_iterations = options_.max_num_iterations;

    // Choose the type of linear solver. For sufficiently large problems, we
    // want to use iterative methods (e.g., Conjugate Gradient or Iterative
    // Schur); however, we only want to use a Schur solver if 3D points are used
    // in the optimization.
    if (positions->size() > kMinNumCamerasForIterativeSolve) {
        if (options_.min_num_points_per_view > 0) {
            solver_options_.linear_solver_type = ceres::ITERATIVE_SCHUR;
            solver_options_.preconditioner_type = ceres::SCHUR_JACOBI;
        }
        else {
            solver_options_.linear_solver_type = ceres::CGNR;
            solver_options_.preconditioner_type = ceres::JACOBI;
        }
    }
    else {
        if (options_.min_num_points_per_view > 0) {
            solver_options_.linear_solver_type = ceres::SPARSE_SCHUR;
        }
        else {
            solver_options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        }
    }

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options_, problem_.get(), &summary);

    LOG(INFO) << summary.FullReport();

    return summary.IsSolutionUsable();
}

bool NonlinearPositionEstimator::EstimateRemainingPositionsInRecon(
    const std::set<ViewId>& fixedViewIds,
    const std::unordered_set<ViewId>& viewIds,
    const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    triangulated_points_.clear();

    Scene subScene;
    _scene.extractSubScene(viewIds, &subScene);

    const auto subViewIds = subScene.viewIds();
    std::unordered_map<ViewId, Vector3d> orientations;
    // get last view id as we assume sequential processing
    for (const auto& viewId : subViewIds) {
        const auto& cam = subScene.view(viewId)->camera();
        orientations[viewId] = cam.orientationAsAngleAxis();
        (*positions)[viewId] = cam.position();
    }

    // assigning fixed views
    _fixedViewIds = fixedViewIds;

    return EstimatePositions(viewPairs, orientations, positions);
}

void NonlinearPositionEstimator::InitializeRandomPositions(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    std::unordered_set<ViewId> constrained_positions;
    constrained_positions.reserve(orientations.size());
    for (const auto& [viewIdPair, _] : *_viewPairs) {
        constrained_positions.insert(viewIdPair.first);
        constrained_positions.insert(viewIdPair.second);
    }

    positions->reserve(orientations.size());
    for (const auto& [viewId, _] : orientations) {
        if (constrained_positions.contains(viewId)) {
            (*positions)[viewId] = 100. * Vector3d::Random();
        }
    }
}

void NonlinearPositionEstimator::AddCameraToCameraConstraints(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    for (const auto& [viewIdPair, info] : *_viewPairs) {
        const auto& viewId1 = viewIdPair.first;
        const auto& viewId2 = viewIdPair.second;
        Vector3d* position1 = con::FindOrNull(*positions, viewId1);
        Vector3d* position2 = con::FindOrNull(*positions, viewId2);

        // Do not add this view pair if one or both of the positions do not
        // exist.
        if (!position1 || !position2) {
            continue;
        }

        // Rotate the relative translation so that it is aligned to the global
        // orientation frame.
        const Vector3d translation_direction = GetRotatedTranslation(
            con::FindOrDie(orientations, viewId1), info.position);

        auto cost_function = PairwiseTranslationError::create(
            translation_direction, 1., info.scale_estimate);

        problem_->AddResidualBlock(
            cost_function, new ceres::HuberLoss(options_.robust_loss_width),
            position1->data(), position2->data());
    }

    VLOG(2) << problem_->NumResidualBlocks()
            << " camera to camera constraints "
               "were added to the position "
               "estimation problem.";
}

void NonlinearPositionEstimator::AddPointToCameraConstraints(
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    const int num_camera_to_camera_constraints = problem_->NumResidualBlocks();
    std::unordered_map<TrackId, bool> tracks_to_add;
    const int num_point_to_camera_constraints =
        FindTracksForProblem(*positions, &tracks_to_add);
    if (num_point_to_camera_constraints == 0) {
        return;
    }

    const double point_to_camera_weight =
        options_.point_to_camera_weight *
        static_cast<double>(num_camera_to_camera_constraints) /
        static_cast<double>(num_point_to_camera_constraints);

    triangulated_points_.reserve(tracks_to_add.size());
    for (const auto& [trackId, estimated] : tracks_to_add) {
        // if track is estimated
        if (estimated) {
            triangulated_points_[trackId] =
                _scene.track(trackId)->position().hnormalized();

            AddTrackToProblem(trackId, orientations, point_to_camera_weight,
                              positions);
        }
        else {
            triangulated_points_[trackId] = 100.0 * Vector3d::Random();
        }
    }

    VLOG(2) << num_point_to_camera_constraints
            << " point to camera constriants "
               "were added to the position "
               "estimation problem.";
}

int NonlinearPositionEstimator::FindTracksForProblem(
    const std::unordered_map<ViewId, Eigen::Vector3d>& positions,
    std::unordered_map<TrackId, bool>* tracks_to_add) const
{
    CHECK_NOTNULL(tracks_to_add)->clear();

    std::unordered_map<ViewId, int> tracksPerView;
    for (const auto& [viewId, _] : positions) {
        tracksPerView[viewId] = 0;
    }

    // Add the tracks that see the most views until each camera has the minimum
    // number of tracks.
    for (const auto& [viewId, _] : positions) {
        const auto* view = _scene.view(viewId);
        if (!view || view->featureCount() < options_.min_num_points_per_view) {
            continue;
        }

        // Get the tracks in sorted order so that we add the tracks that see the
        // most cameras first.
        const auto sortedTrackIds =
            GetTracksSortedByNumViews(_scene, *view, *tracks_to_add);

        auto& trackCount = tracksPerView[viewId];
        for (const auto& trackId : sortedTrackIds) {
            if (trackCount >= options_.min_num_points_per_view) {
                break;
            }

            // Update the number of point to camera constraints for each camera.
            (*tracks_to_add)[trackId] = _scene.track(trackId)->estimated();
            for (const auto& viewId : _scene.track(trackId)->viewIds()) {
                if (positions.contains(viewId)) {
                    ++trackCount;
                }
            }
        }
    }

    int num_point_to_camera_constraints = 0;
    for (const auto& [_, count] : tracksPerView) {
        num_point_to_camera_constraints += count;
    }

    return num_point_to_camera_constraints;
}

std::vector<TrackId> NonlinearPositionEstimator::GetTracksSortedByNumViews(
    const Scene& scene, const View& view,
    const std::unordered_map<TrackId, bool>& existing_tracks) const
{
    std::vector<std::pair<TrackId, int>> viewsPerTrack;
    viewsPerTrack.reserve(view.featureCount());
    for (const auto& trackId : view.trackIds()) {
        if (const auto* track = scene.track(trackId);
            track && !existing_tracks.contains(trackId)) {
            viewsPerTrack.emplace_back(trackId, track->viewCount());
        }
    }

    // Return an empty array if no tracks could be found for this view.
    if (viewsPerTrack.empty()) {
        return {};
    }

    std::vector<TrackId> sorted_tracks(viewsPerTrack.size());
    // Sort the tracks by the number of views. Only sort the first few tracks
    // since those are the ones that will be added to the problem.
    const int num_tracks_to_sort =
        std::min(static_cast<int>(viewsPerTrack.size()),
                 options_.min_num_points_per_view);
    std::partial_sort(viewsPerTrack.begin(),
                      viewsPerTrack.begin() + num_tracks_to_sort,
                      viewsPerTrack.end(),
                      [](const std::pair<TrackId, int>& t1,
                         const std::pair<TrackId, int>& t2) {
                          return t1.second > t2.second;
                      });

    for (int i = 0; i < num_tracks_to_sort; i++) {
        sorted_tracks[i] = viewsPerTrack[i].first;
    }
    return sorted_tracks;
}

void NonlinearPositionEstimator::AddTrackToProblem(
    TrackId trackId,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    double point_to_camera_weight,
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    // For each view in the track add the point to camera correspondences.
    for (const auto& viewId : _scene.track(trackId)->viewIds()) {
        if (!positions->contains(viewId)) {
            continue;
        }

        Vector3d& camera_position = con::FindOrDie(*positions, viewId);
        Vector3d& point = con::FindOrDie(triangulated_points_, trackId);

        // Rotate the feature ray to be in the global orientation frame.
        const Vector3d feature_ray = GetRotatedFeatureRay(
            _scene.view(viewId)->camera(), con::FindOrDie(orientations, viewId),
            *_scene.view(viewId)->featureOf(trackId));

        // Rotate the relative translation so that it is aligned to the global
        // orientation frame.
        auto cost_function = PairwiseTranslationError::create(
            feature_ray, point_to_camera_weight, -1.);

        // Add the residual block
        problem_->AddResidualBlock(
            cost_function, new ceres::HuberLoss(options_.robust_loss_width),
            camera_position.data(), point.data());
    }
}

void NonlinearPositionEstimator::AddCamerasAndPointsToParameterGroups(
    std::unordered_map<ViewId, Eigen::Vector3d>* positions)
{
    CHECK(!triangulated_points_.empty())
        << "Cannot set the Ceres parameter groups for Schur based solvers "
           "because there are no triangulated points.";

    // Create a custom ordering for Schur-based problems.
    solver_options_.linear_solver_ordering.reset(
        new ceres::ParameterBlockOrdering);
    auto parameter_ordering = solver_options_.linear_solver_ordering.get();
    // Add point parameters to group 0.
    for (auto& [_, point] : triangulated_points_) {
        parameter_ordering->AddElementToGroup(point.data(), 0);
    }

    // Add camera parameters to group 1.
    for (auto& [_, position] : *positions) {
        parameter_ordering->AddElementToGroup(position.data(), 1);
    }
}

} // namespace tl
