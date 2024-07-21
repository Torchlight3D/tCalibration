#include "globalreconstruction.h"

#include <glog/logging.h>

#include <tCore/ContainerUtils>
#include <tCore/Timer>
#include <tMvs/Scene>
#include <tMvs/ViewGraph>
#include <tMvs/BA/BundleAdjustment>
#include <tMvs/Poses/EstimateRotationHybrid>
#include <tMvs/Poses/EstimateRotationLagrange>
#include <tMvs/Poses/EstimateRotationLinear>
#include <tMvs/Poses/EstimateRotationNonlinear>
#include <tMvs/Poses/EstimateRotationRobust>

#include "optimzerelativeposition.h"
#include "trackestimator.h"
#include "threadpool.h"
#include "selecttracks.h"
#include "selectviewpairs.h"

namespace tl {

using Eigen::Vector3d;

namespace {

void SetUnderconstrainedAsUnestimated(Scene* scene)
{
    int num_underconstrained_views = -1;
    int num_underconstrained_tracks = -1;
    while (num_underconstrained_views != 0 &&
           num_underconstrained_tracks != 0) {
        num_underconstrained_views =
            scene->setUnderconstrainedViewsUnestimated();
        num_underconstrained_tracks =
            scene->setUnderconstrainedTracksUnestimated();
    }
}

std::vector<Feature2D2D> normalizedFeatureMatches(const View& view1,
                                                  const View& view2)
{
    const auto& camera1 = view1.camera();
    const auto& camera2 = view2.camera();

    const auto trackIds = view1.trackIds();

    std::vector<Feature2D2D> matches;
    matches.reserve(trackIds.size());
    for (const auto& trackId : trackIds) {
        const auto* feat2 = view2.featureOf(trackId);
        if (!feat2) {
            continue;
        }

        const auto* feat1 = view1.featureOf(trackId);

        Feature2D2D match;
        match.feature1 = {
            camera1.pixelToNormalizedCoordinates(feat1->pos).hnormalized()};
        match.feature2 = {
            camera2.pixelToNormalizedCoordinates(feat2->pos).hnormalized()};
        matches.push_back(match);
    }

    return matches;
}

} // namespace

struct TimeSummary
{
    double initial_view_graph_filtering_time = 0.;
    double camera_intrinsics_calibration_time = 0.;
    double rotation_estimation_time = 0.;
    double rotation_filtering_time = 0.;
    double relative_translation_optimization_time = 0.;
    double relative_translation_filtering_time = 0.;
    double position_estimation_time = 0.;
};

GlobalReconstruction::GlobalReconstruction(const Options& options)
    : Reconstruction(), _opts(options)
{
}

Reconstruction::Summary GlobalReconstruction::Estimate(ViewGraph* viewGraph,
                                                       Scene* scene)
{
    CHECK_NOTNULL(viewGraph);
    CHECK_NOTNULL(scene);

    _scene = scene;
    _viewGraph = viewGraph;
    _orientations.clear();
    _positions.clear();

    Summary summary;
    TimeSummary timeSummary;
    Timer total_timer;
    Timer timer;

    // Step 1. Filter the initial view graph and remove any bad view pairs.
    LOG(INFO) << "Filtering the initial view graph.";
    timer.reset();

    if (!filterInitialViewGraph()) {
        LOG(ERROR) << "Insufficient view pairs to perform the following "
                      "reconstruction.";
        return summary;
    }

    timeSummary.initial_view_graph_filtering_time = timer.elapseInSecond();

    // Step 2. Calibrate any uncalibrated cameras.
    LOG(INFO) << "Calibrating any uncalibrated cameras.";
    timer.reset();

    initializeCameraIntrinsics();

    timeSummary.camera_intrinsics_calibration_time = timer.elapseInSecond();
    summary.camera_intrinsics_calibration_time =
        timeSummary.camera_intrinsics_calibration_time;

    // Step 3a. Estimate global rotations.
    LOG(INFO) << "Estimating the global rotations of all views.";
    timer.reset();

    if (!estimateGlobalRotations()) {
        LOG(ERROR) << "Failed to estimate global rotation.";
        return summary;
    }

    timeSummary.rotation_estimation_time = timer.elapseInSecond();

    // Step 3b. Filter bad rotations.
    LOG(INFO) << "Filtering any bad rotation estimations.";
    timer.reset();

    filterRotations();

    timeSummary.rotation_filtering_time = timer.elapseInSecond();

    // (Optional) Step 4. Optimize relative position.
    if (_opts.refine_relative_translations_after_rotation_estimation) {
        if (_opts.global_position_estimator_type ==
            GlobalPositionEstimatorType::LiGT) {
            LOG(INFO) << "LiGT selected. "
                         "Skip pairwise translation estimation and filtering.";
        }
        else {
            // Step 4a. Optimize relative translations.
            LOG(INFO) << "Optimizing the (pairwise) relative translations.";
            timer.reset();

            optimizeRelativeTranslations();

            timeSummary.relative_translation_optimization_time =
                timer.elapseInSecond();

            // Step 4b. Filter bad relative translations.
            LOG(INFO) << "Filtering any bad relative translations.";
            timer.reset();

            filterRelativeTranslations();

            timeSummary.relative_translation_filtering_time =
                timer.elapseInSecond();
        }
    }

    // Step 5. Estimate global positions.
    LOG(INFO) << "Estimating the global positions of all the views.";
    timer.reset();

    if (!estimateGlobalPositions()) {
        LOG(WARNING) << "Failed to estimate global position.";
        return summary;
    }

    LOG(INFO) << _positions.size() << " view positions are estimated.";
    timeSummary.position_estimation_time = timer.elapseInSecond();
    summary.pose_estimation_time =
        timeSummary.rotation_estimation_time +
        timeSummary.rotation_filtering_time +
        timeSummary.relative_translation_optimization_time +
        timeSummary.relative_translation_filtering_time +
        timeSummary.position_estimation_time;

    // Update the estimated poses to the scene.
    _scene->updateViewPoses(_orientations, _positions);

    // Always triangulate once, then retriangulate and remove outliers depending
    // on the reconstruciton estimator options.
    for (auto i{0}; i < _opts.num_retriangulation_iterations + 1; ++i) {
        // Step 6. Triangulate features.
        LOG(INFO) << "Triangulating all features.";
        timer.reset();

        estimateStructure();

        summary.triangulation_time += timer.elapseInSecond();

        SetUnderconstrainedAsUnestimated(_scene);

        // Do a single step of bundle adjustment where only the camera positions
        // and 3D points are refined. This is only done for the very first
        // bundle adjustment iteration.
        if (i == 0 &&
            _opts
                .refine_camera_positions_and_points_after_position_estimation) {
            LOG(INFO)
                << "Performing partial bundle adjustment to optimize only the "
                   "camera positions and 3d points.";
            timer.reset();

            bundleAdjustPositionsAndLandmarks();

            summary.bundle_adjustment_time += timer.elapseInSecond();
        }

        // Step 7. Bundle Adjustment.
        LOG(INFO) << "Performing bundle adjustment.";
        timer.reset();

        if (!bundleAdjustment()) {
            LOG(WARNING) << "Bundle adjustment failed!";
            return summary;
        }

        summary.bundle_adjustment_time += timer.elapseInSecond();

        const int num_points_removed = _scene->setOutlierTracksUnestimated(
            _opts.max_reprojection_error_in_pixels,
            _opts.min_triangulation_angle_degrees);
        LOG(INFO) << num_points_removed << " outlier points were removed.";
    }

    // Set the output parameters.
    summary.estimated_views = _scene->estimatedViewIds();
    summary.estimated_tracks = _scene->estimatedTrackIds();
    summary.success = true;
    summary.total_time = total_timer.elapseInSecond();
    summary.message = [&] {
        std::ostringstream oss;
        oss << "Global Reconstruction Estimator timings:"
               "\n\t"
               "Initial view graph filtering time: "
            << timeSummary.initial_view_graph_filtering_time
            << "\n\t"
               "Camera intrinsic calibration time: "
            << summary.camera_intrinsics_calibration_time
            << "\n\t"
               "Rotation estimation time: "
            << timeSummary.rotation_estimation_time
            << "\n\t"
               "Rotation filtering time: "
            << timeSummary.rotation_filtering_time
            << "\n\t"
               "Relative translation optimization time: "
            << timeSummary.relative_translation_optimization_time
            << "\n\t"
               "Relative translation filtering time: "
            << timeSummary.relative_translation_filtering_time
            << "\n\t"
               "Position estimation time: "
            << timeSummary.position_estimation_time;
        return oss.str();
    }();

    return summary;
}

bool GlobalReconstruction::filterInitialViewGraph()
{
    // Remove any view pairs that do not have a sufficient number of inliers.
    std::unordered_set<ViewIdPair> removeIds;
    for (const auto& [id, info] : _viewGraph->edgesAndInfo()) {
        if (info.num_verified_matches < _opts.min_num_two_view_inliers) {
            removeIds.insert(id);
        }
    }

    for (const auto& [viewId1, viewId2] : removeIds) {
        _viewGraph->removeViewPair(viewId1, viewId2);
    }

    // Only reconstruct the largest connected component.
    _viewGraph->removeDisconnectedViewPairs();
    return _viewGraph->numViewPairs() >= 1;
}

// NOTE: Not ACTUALLY initialize camera intrinsics in a convention context, but
// set the initialized camera parameters into the views.
void GlobalReconstruction::initializeCameraIntrinsics()
{
    setCameraIntrinsicsFromMetaData(_scene);
}

bool GlobalReconstruction::estimateGlobalRotations()
{
    const auto& viewPairs = _viewGraph->viewPairs();

    std::unique_ptr<RotationEstimator> estimator;
    switch (_opts.global_rotation_estimator_type) {
        case GlobalRotationEstimatorType::Robust: {
            _viewGraph->OrientationsFromMaximumSpanningTree(&_orientations);

            RobustRotationEstimator::Options opts;
            estimator.reset(new RobustRotationEstimator(opts));
            break;
        }
        case GlobalRotationEstimatorType::Nonlinear: {
            _viewGraph->OrientationsFromMaximumSpanningTree(&_orientations);

            estimator.reset(new NonlinearRotationEstimator());
            break;
        }
        case GlobalRotationEstimatorType::Linear: {
            // Set the constructor variable to true to weigh each term by the
            // inlier count.
            estimator.reset(new LinearRotationEstimator());
            break;
        }
        case GlobalRotationEstimatorType::LagrangeDuality: {
            _viewGraph->OrientationsFromMaximumSpanningTree(&_orientations);

            estimator.reset(new LagrangeDualRotationEstimator());
            break;
        }
        case GlobalRotationEstimatorType::Hybrid: {
            _viewGraph->OrientationsFromMaximumSpanningTree(&_orientations);
            estimator.reset(new HybridRotationEstimator());
            break;
        }
        default: {
            LOG(FATAL) << "Unsupported global rotation estimation algorithm."
                       << static_cast<int>(
                              _opts.global_rotation_estimator_type);
        }
    }

    return estimator->EstimateRotations(viewPairs, &_orientations);
}

void GlobalReconstruction::filterRotations()
{
    FilterViewPairs::ByOrientations(
        _orientations, _opts.rotation_filtering_max_difference_degrees,
        _viewGraph);

    // Remove any disconnected views from the estimation.
    const auto removedViewIds = _viewGraph->removeDisconnectedViewPairs();
    for (const auto& viewId : removedViewIds) {
        _orientations.erase(viewId);
    }
}

void GlobalReconstruction::optimizeRelativeTranslations()
{
    const auto& viewPairs = _viewGraph->edgesAndInfo();

    ThreadPool pool{_opts.num_threads};
    for (const auto& [id, _] : viewPairs) {
        const auto& viewId1 = id.first;
        const auto& viewId2 = id.second;

        const auto* view1 = _scene->view(viewId1);
        const auto* view2 = _scene->view(viewId2);
        const auto matches = normalizedFeatureMatches(*view1, *view2);

        auto* info = _viewGraph->rEdge(viewId1, viewId2);
        pool.Add(OptimizeRelativePositionWithKnownRotation, matches,
                 con::FindOrDie(_orientations, viewId1),
                 con::FindOrDie(_orientations, viewId2), &info->position);
    }
}

void GlobalReconstruction::filterRelativeTranslations()
{
    if (_opts.extract_maximal_rigid_subgraph) {
        LOG(INFO)
            << "Extracting maximal rigid component of viewing graph to "
               "determine which cameras are well-constrained for position "
               "estimation.";
        ExtractMaximallyParallelRigidSubgraph(_orientations, _viewGraph);
    }

    // Filter potentially bad relative translations.
    if (_opts.filter_relative_translations_with_1dsfm) {
        LOG(INFO) << "Filtering relative translations with 1DSfM filter.";

        FilterViewPairs::ByRelativeTranslationOptions opts;
        opts.num_threads = _opts.num_threads;
        opts.num_iterations = _opts.translation_filtering_num_iterations;
        opts.translation_projection_tolerance =
            _opts.translation_filtering_projection_tolerance;

        FilterViewPairs::ByRelativeTranslations(opts, _orientations,
                                                _viewGraph);
    }

    const auto removedViewIds = _viewGraph->removeDisconnectedViewPairs();
    for (const auto& viewId : removedViewIds) {
        _orientations.erase(viewId);
    }
}

bool GlobalReconstruction::estimateGlobalPositions()
{
    const auto& viewPairs = _viewGraph->viewPairs();

    std::unique_ptr<PositionEstimator> estimator;
    switch (_opts.global_position_estimator_type) {
        case GlobalPositionEstimatorType::LUD: {
            estimator.reset(new LeastUnsquaredDeviationPositionEstimator(
                _opts.least_unsquared_deviation_position_estimator_options));
            break;
        }
        case GlobalPositionEstimatorType::Nonlinear: {
            auto opts = _opts.nonlinear_position_estimator_options;
            opts.rng = _opts.rng;
            opts.num_threads = _opts.num_threads;
            estimator.reset(new NonlinearPositionEstimator(opts, *_scene));
            break;
        }
        case GlobalPositionEstimatorType::Linear: {
            auto opts = _opts.linear_triplet_position_estimator_options;
            opts.num_threads = _opts.num_threads;
            estimator.reset(new LinearPositionEstimator(opts, *_scene));
            break;
        }
        case GlobalPositionEstimatorType::LiGT: {
            auto opts = _opts.ligt_position_estimator_options;
            opts.num_threads = _opts.num_threads;
            estimator.reset(new LiGTPositionEstimator(opts, *_scene));
            break;
        }
        default: {
            LOG(FATAL) << "Invalid type of global position estimation chosen.";
            break;
        }
    }

    return estimator->EstimatePositions(viewPairs, _orientations, &_positions);
}

void GlobalReconstruction::estimateStructure()
{
    TrackEstimator::Options opts;
    opts.max_acceptable_reprojection_error_pixels =
        _opts.triangulation_max_reprojection_error_in_pixels;
    opts.min_triangulation_angle_degrees =
        _opts.min_triangulation_angle_degrees;
    opts.bundle_adjustment = _opts.bundle_adjust_tracks;
    opts.ba_options = _opts.toBundleAdjustmentOptions(0);
    opts.ba_options.num_threads = 1;
    opts.ba_options.verbose = false;
    opts.num_threads = _opts.num_threads;
    opts.triangulation_method = _opts.triangulation_method;

    TrackEstimator estimator{opts, _scene};
    [[maybe_unused]] const auto summary = estimator.EstimateAllTracks();
}

bool GlobalReconstruction::bundleAdjustment()
{
    std::vector<TrackId> selectedTrackIds = _scene->estimatedTrackIds();
    if (_opts.subsample_tracks_for_bundle_adjustment) {
        SelectGoodTracks::Options opts;
        opts.maxTrackObservedCount =
            _opts.track_subset_selection_long_track_length_threshold;
        opts.imageCellSize = _opts.track_selection_image_grid_cell_size_pixels;
        opts.minTrackPerView = _opts.min_num_optimized_tracks_per_view;

        const auto goodTrackIds = SelectGoodTracks::Select(opts, *_scene);

        if (!goodTrackIds.empty()) {
            // Set all tracks that were not chosen for BA to be unestimated so
            // that they do not affect the bundle adjustment optimization.
            _scene->setTracksInViewsUnestimated(_scene->viewIds(),
                                                goodTrackIds);
            // TODO: use swap
            selectedTrackIds = goodTrackIds;
        }
    }

    LOG(INFO) << "Selected " << selectedTrackIds.size()
              << " tracks to optimize.";

    const auto opts = _opts.toBundleAdjustmentOptions(_positions.size());

    const auto summary = BundleAdjustPartialScene(
        opts, _scene->estimatedViewIds(), selectedTrackIds, _scene);

    return summary.success;
}

bool GlobalReconstruction::bundleAdjustPositionsAndLandmarks()
{
    std::vector<TrackId> selectedTrackIds = _scene->estimatedTrackIds();
    if (_opts.subsample_tracks_for_bundle_adjustment) {
        SelectGoodTracks::Options opts;
        opts.maxTrackObservedCount =
            _opts.track_subset_selection_long_track_length_threshold;
        opts.imageCellSize = _opts.track_selection_image_grid_cell_size_pixels;
        opts.minTrackPerView = _opts.min_num_optimized_tracks_per_view;

        const auto goodTrackIds = SelectGoodTracks::Select(opts, *_scene);
        if (!goodTrackIds.empty()) {
            // TODO: use swap
            selectedTrackIds = goodTrackIds;
        }
    }

    LOG(INFO) << "Selected " << selectedTrackIds.size()
              << " tracks to optimize.";

    auto opts = _opts.toBundleAdjustmentOptions(_positions.size());
    opts.constant_camera_orientation = true;
    opts.constant_camera_position = false;
    opts.intrinsics_to_optimize = OptimizeIntrinsicsType::None;

    const auto summary = BundleAdjustPartialScene(
        opts, _scene->estimatedViewIds(), selectedTrackIds, _scene);
    return summary.success;
}

} // namespace tl
