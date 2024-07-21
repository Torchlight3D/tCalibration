#include "incrementalreconstruction.h"

#include <ceres/rotation.h>
#include <glog/logging.h>

#include <tCore/Timer>
#include <tMVS/Scene>
#include <tMVS/ViewGraph>

#include "selecttracks.h"
#include "trackestimator.h"
#include "visibilitypyramid.h"
#include "localizeviewtoscene.h"

namespace tl {

namespace {

} // namespace

SacParameters IncrementalReconstruction::Options::toRansacParameters() const
{
    SacParameters params;
    params.rng = rng;
    params.failure_probability = 1. - ransac_confidence;
    params.min_iterations = ransac_min_iterations;
    params.max_iterations = ransac_max_iterations;
    params.use_mle = ransac_use_mle;
    params.use_lo = ransac_use_lo;
    params.lo_start_iterations = ransac_lo_start_iterations;
    return params;
}

IncrementalReconstruction::IncrementalReconstruction(const Options& opts)
    : Reconstruction()
{
    CHECK_LE(opts.multiple_view_localization_ratio, 1.0)
        << "The multiple view localization ratio must be between 0 and 1.0.";
    CHECK_GE(opts.multiple_view_localization_ratio, 0.0)
        << "The multiple view localization ratio must be between 0 and 1.0.";
    CHECK_GE(opts.full_bundle_adjustment_growth_percent, 0.0)
        << "The bundle adjustment growth percent must be greater than 0 "
           "percent.";
    CHECK_GE(opts.partial_bundle_adjustment_num_views, 0)
        << "The bundle adjustment growth percent must be greater than 0 "
           "percent.";

    _opts = opts;
}

Reconstruction::Summary IncrementalReconstruction::Estimate(
    ViewGraph* viewGraph, Scene* scene)
{
    // Setup configs and cached data
    _scene = scene;
    _viewGraph = viewGraph;

    const auto viewIds = _viewGraph->viewIds();
    unlocalized_views_.reserve(viewIds.size());
    for (const auto& viewId : viewIds) {
        if (const auto* view = _scene->view(viewId);
            view && !view->estimated()) {
            unlocalized_views_.insert(viewId);
        }
    }

    reconstructed_views_.clear();

    num_optimized_views_ = 0;

    // Algorithm starts from here
    Summary summary;

    Timer total_timer;
    Timer timer;
    double time_to_find_initial_seed = 0;

    // Step 0: Set the known camera intrinsics.
    timer.reset();
    setCameraIntrinsicsFromMetaData(_scene);
    summary.camera_intrinsics_calibration_time = timer.elapseInSecond();

    // Steps 1 - 3: Choose an initial camera pair to reconstruct if the
    // reconstruction is not already initialized.
    const int num_estimated_tracks = _scene->estimatedTrackIds().size();
    [[maybe_unused]] const int num_estimated_views =
        _scene->estimatedViewIds().size();
    if (num_estimated_tracks < _opts.min_num_absolute_pose_inliers ||
        num_estimated_tracks < 2) {
        timer.reset();
        if (!ChooseInitialViewPair()) {
            LOG(ERROR) << "Could not find a suitable initial pair for starting "
                          "incremental SfM!";
            summary.success = false;
            return summary;
        }
        time_to_find_initial_seed = timer.elapseInSecond();
    }

    // Local view options
    LocalizeViewToReconstructionOptions locateViewOpts;
    locateViewOpts.reprojection_error_threshold_pixels =
        _opts.absolute_pose_reprojection_error_threshold;
    locateViewOpts.ransac_params = _opts.toRansacParameters();
    locateViewOpts.bundle_adjust_view = true;
    locateViewOpts.ba_options = _opts.toBundleAdjustmentOptions(0);
    locateViewOpts.ba_options.verbose = false;
    locateViewOpts.min_num_inliers = _opts.min_num_absolute_pose_inliers;
    locateViewOpts.pnp_type = _opts.localization_pnp_type;

    // Triangulation options
    TrackEstimator::Options triangulationOpts;
    triangulationOpts.max_acceptable_reprojection_error_pixels =
        _opts.triangulation_max_reprojection_error_in_pixels;
    triangulationOpts.min_triangulation_angle_degrees =
        _opts.min_triangulation_angle_degrees;
    triangulationOpts.bundle_adjustment = _opts.bundle_adjust_tracks;
    triangulationOpts.ba_options = _opts.toBundleAdjustmentOptions(0);
    triangulationOpts.ba_options.num_threads = 1;
    triangulationOpts.ba_options.verbose = false;
    triangulationOpts.num_threads = _opts.num_threads;

    // Try to add as many views as possible to the reconstruction until no more
    // views can be localized.
    std::vector<ViewId> views_to_localize;
    int failed_localization_attempts = -1;
    while (!unlocalized_views_.empty() &&
           failed_localization_attempts != views_to_localize.size()) {
        failed_localization_attempts = 0;
        views_to_localize.clear();

        // Step 4: Localize new views.
        // Compute the 2D-3D point count to determine which views should be
        // localized.
        timer.reset();
        FindViewsToLocalize(&views_to_localize);
        summary.pose_estimation_time += timer.elapseInSecond();
        LOG(INFO) << "Will try to localize " << views_to_localize.size()
                  << " views.";

        // Attempt to localize all candidate views and estimate new 3D
        // points. Bundle Adjustment is run as either partial or full BA
        // depending on the current state of the reconstruction.
        for (const auto& viewId : views_to_localize) {
            timer.reset();
            SacSummary unused_ransac_summary;
            if (!LocalizeViewToReconstruction(viewId, locateViewOpts, _scene,
                                              &unused_ransac_summary)) {
                ++failed_localization_attempts;
                continue;
            }
            summary.pose_estimation_time += timer.elapseInSecond();

            reconstructed_views_.push_back(viewId);
            unlocalized_views_.erase(viewId);

            // Remove any tracks that have very bad 3D point reprojections after
            // the new view has been merged. This can happen when a new
            // observation of a 3D point has a very high reprojection error in
            // the newly localized view.
            const auto trackIdsInNewView =
                _scene->view(reconstructed_views_.back())->trackIds();
            removeOutlierTracks(
                trackIdsInNewView,
                triangulationOpts.max_acceptable_reprojection_error_pixels);

            // Step 5: Estimate new 3D points. and Step 6: Bundle adjustment.
            bool ba_success = false;
            if (UnoptimizedGrowthPercentage() <
                _opts.full_bundle_adjustment_growth_percent) {
                // Step 5: Perform triangulation on the most recent view.
                timer.reset();
                EstimateStructure(reconstructed_views_.back());
                summary.triangulation_time += timer.elapseInSecond();

                // Step 6: Partial Bundle Adjustment.
                timer.reset();
                ba_success = partialBundleAdjustment();
                summary.bundle_adjustment_time += timer.elapseInSecond();
            }
            else {
                // Step 5: Perform triangulation on all views.
                {
                    timer.reset();
                    TrackEstimator triangulator{triangulationOpts, _scene};
                    [[maybe_unused]] const auto _ =
                        triangulator.EstimateAllTracks();
                    summary.triangulation_time += timer.elapseInSecond();
                }

                // Step 6: Full Bundle Adjustment.
                timer.reset();
                ba_success = fullBundleAdjustment();
                summary.bundle_adjustment_time += timer.elapseInSecond();
            }

            SetUnderconstrainedAsUnestimated();

            if (!ba_success) {
                LOG(WARNING) << "Bundle adjustment failed!";
                summary.success = false;
                return summary;
            }

            // Force exiting the loop so that the next best view scores are
            // recomputed.
            break;
        }
    }

    // Set the output parameters.
    summary.estimated_views = _scene->estimatedViewIds();
    summary.estimated_tracks = _scene->estimatedTrackIds();
    summary.success = true;
    summary.total_time = total_timer.elapseInSecond();

    std::ostringstream oss;
    oss << "Incremental Reconstruction Estimator timings:"
        << "\n\tTime to find an initial seed for the reconstruction: "
        << time_to_find_initial_seed;
    summary.message = oss.str();

    return summary;
}

void IncrementalReconstruction::initializeCamerasInViewPair(
    const ViewIdPair& viewPairId)
{
    const auto& viewId1 = viewPairId.first;
    const auto& viewId2 = viewPairId.second;

    auto* view1 = _scene->rView(viewId1);
    auto* view2 = _scene->rView(viewId2);
    const auto* info = _viewGraph->edge(viewId1, viewId2);

    // Pose of camera1 identity
    auto& camera1 = view1->rCamera();
    camera1.setFocalLength(info->focalLength1);
    auto& camera2 = view2->rCamera();
    camera2.setOrientationFromAngleAxis(info->rotation);
    camera2.setPosition(info->position);
    camera2.setFocalLength(info->focalLength2);

    view1->setEstimated(true);
    view2->setEstimated(true);
}

bool IncrementalReconstruction::ChooseInitialViewPair()
{
    // Sort the view pairs by the number of geometrically verified matches.
    constexpr int kMinNumInitialTracks = 100;
    const auto candidates =
        OrderViewPairsByInitializationCriterion(kMinNumInitialTracks);

    if (candidates.empty()) {
        return false;
    }

    // Try to initialize the reconstruction from the candidate view pairs. An
    // initial seed is only considered valid if the baseline relative to the 3D
    // point depths is sufficient. This robustness is measured by the angle of
    // all 3D points.
    for (const auto& viewPairId : candidates) {
        // Set all values as unestimated and try to use the next candidate pair.
        _scene->resetEstimated();

        // Initialize the camera poses of the initial views and set the two
        // views to estimated.
        initializeCamerasInViewPair(viewPairId);

        // Estimate 3D structure of the scene.
        EstimateStructure(viewPairId.first);

        // If we did not triangulate enough tracks then skip this view and try
        // another.
        auto estimatedTrackIds = _scene->estimatedTrackIds();

        if (estimatedTrackIds.size() < kMinNumInitialTracks) {
            continue;
        }

        // Bundle adjustment on the 2-view reconstruction.
        if (!fullBundleAdjustment()) {
            continue;
        }

        // If we triangulated enough 3D points then return.  Otherwise, try the
        // next view pair as the seed for the initial reconstruction.
        estimatedTrackIds.clear();
        estimatedTrackIds = _scene->estimatedTrackIds();

        if (estimatedTrackIds.size() > kMinNumInitialTracks) {
            reconstructed_views_.push_back(viewPairId.first);
            reconstructed_views_.push_back(viewPairId.second);
            unlocalized_views_.erase(viewPairId.first);
            unlocalized_views_.erase(viewPairId.second);

            return true;
        }
    }

    return false;
}

std::vector<ViewIdPair>
IncrementalReconstruction::OrderViewPairsByInitializationCriterion(
    int minVerifiedMatch) const
{
    const auto& viewPairs = _viewGraph->edgesAndInfo();

    // Collect the number of inliers for each view pair. The tuples store:
    //     # homography inliers, negative of essential matrix inliers,
    //     ViewIdPair
    //
    // NOTE: We store the negative of the essential matrix inliers because we
    // want a view pair with the fewest homography inliers but the greatest
    // number of essential matrix inliers. This situation only arises if there
    // were a tiebreaker in the number of homography inliers, or if (for some
    // unknown reason) the number of homography inliers is set to 0 for all view
    // pairs.
    std::vector<std::tuple<int, int, ViewIdPair>> stats;
    stats.reserve(viewPairs.size());
    for (const auto& [id, info] : viewPairs) {
        // TODO: We also prefer view pairs with known intrinsics.
        if (info.num_verified_matches > minVerifiedMatch) {
            stats.emplace_back(info.num_homography_inliers,
                               -info.num_verified_matches, id);
        }
    }

    // Sort the views to find the ones that are least well-modelled by a
    // homography.
    std::sort(stats.begin(), stats.end());

    std::vector<ViewIdPair> viewPairIds;
    viewPairIds.reserve(viewPairs.size());
    for (const auto& [_1, _2, id] : stats) {
        viewPairIds.push_back(id);
    }

    return viewPairIds;
}

void IncrementalReconstruction::FindViewsToLocalize(
    std::vector<ViewId>* views_to_localize) const
{
    // We localize all views that observe 75% or more of the best visibility
    // score.
    constexpr int kMinObservedTrackInView = 30;
    constexpr int kNumPyramidLevels = 6;

    // Determine the number of estimated tracks that each view observes.
    // Do not consider estimated views since they have already been
    // localized.
    std::vector<std::pair<int, ViewId>> next_best_view_scores;
    next_best_view_scores.reserve(unlocalized_views_.size());
    for (const auto& viewId : unlocalized_views_) {
        const auto* view = _scene->view(viewId);
        const auto& camera = view->camera();

        // Count the number of estimated tracks for this view.
        const auto trackIds = view->trackIds();
        VisibilityPyramid pyramid{camera.imageWidth(), camera.imageHeight(),
                                  kNumPyramidLevels};
        auto estimatedTrackCount{0};
        for (const auto& trackId : trackIds) {
            if (_scene->track(trackId)->estimated()) {
                ++estimatedTrackCount;
                pyramid.addPoint(view->featureOf(trackId)->pos);
            }
        }

        // If enough 3d points are observed then add the visibility score to the
        // candidate views to localize.
        if (estimatedTrackCount >= kMinObservedTrackInView) {
            next_best_view_scores.emplace_back(pyramid.computeScore(), viewId);
        }
    }

    // Sort such that the best score is at the front.
    std::sort(next_best_view_scores.begin(), next_best_view_scores.end(),
              std::greater<std::pair<int, ViewId>>());
    for (int i = 0; i < next_best_view_scores.size(); i++) {
        views_to_localize->emplace_back(next_best_view_scores[i].second);
    }
}

void IncrementalReconstruction::EstimateStructure(ViewId viewId)
{
    // Estimate all tracks.
    // FIXME: Duplicated code
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

    TrackEstimator triangulator{opts, _scene};
    const auto trackIdsInView = _scene->view(viewId)->trackIds();
    [[maybe_unused]] const auto summary =
        triangulator.EstimateTracks(std::unordered_set<TrackId>{
            trackIdsInView.cbegin(), trackIdsInView.cend()});
}

double IncrementalReconstruction::UnoptimizedGrowthPercentage() const
{
    return 100. * (reconstructed_views_.size() - num_optimized_views_) /
           num_optimized_views_;
}

bool IncrementalReconstruction::fullBundleAdjustment()
{
    LOG(INFO) << "Start full bundle adjustment on the entire scene.";

    // Set up the BA options.
    auto baOpts = _opts.toBundleAdjustmentOptions(reconstructed_views_.size());

    // Inner iterations are not really needed for incremental SfM because we are
    // *hopefully* already starting at a good local minima. Inner iterations are
    // disabled because they slow down BA a lot.
    baOpts.use_inner_iterations = false;

    // If desired, select good tracks to optimize for BA. This dramatically
    // reduces the number of parameters in bundle adjustment, and does a decent
    // job of filtering tracks with outliers that may slow down the nonlinear
    // optimization.
    auto selectedTrackIds = _scene->estimatedTrackIds();
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

    const auto summary = BundleAdjustPartialScene(
        baOpts, _scene->estimatedViewIds(), selectedTrackIds, _scene);
    num_optimized_views_ = reconstructed_views_.size();

    const auto trackIds = _scene->trackIds();
    removeOutlierTracks(trackIds, _opts.max_reprojection_error_in_pixels);

    return summary.success;
}

bool IncrementalReconstruction::partialBundleAdjustment()
{
    // Partial bundle adjustment only only the k most recently added views that
    // have not been optimized by full BA.
    const int partial_ba_size =
        std::min(static_cast<int>(reconstructed_views_.size()),
                 _opts.partial_bundle_adjustment_num_views);

    LOG(INFO) << "Running partial bundle adjustment on " << partial_ba_size
              << " views.";

    // Set up the BA options.
    auto baOpts = _opts.toBundleAdjustmentOptions(partial_ba_size);

    // Inner iterations are not really needed for incremental SfM because we are
    // *hopefully* already starting at a good local minima. Inner iterations are
    // disabled because they slow down BA a lot.
    baOpts.use_inner_iterations = false;
    baOpts.verbose = VLOG_IS_ON(2);

    // If the model has grown sufficiently then run BA on the entire
    // model. Otherwise, run partial BA.

    // Get the views to optimize for partial BA.
    std::unordered_set<ViewId> views_to_optimize(
        reconstructed_views_.end() - partial_ba_size,
        reconstructed_views_.end());

    const std::vector<ViewId> selectedViewIds{views_to_optimize.cbegin(),
                                              views_to_optimize.cend()};

    // If desired, select good tracks to optimize for BA. This dramatically
    // reduces the number of parameters in bundle adjustment, and does a decent
    // job of filtering tracks with outliers that may slow down the nonlinear
    // optimization.
    std::vector<TrackId> selectedTrackIds;
    if (_opts.subsample_tracks_for_bundle_adjustment) {
        SelectGoodTracks::Options opts;
        opts.maxTrackObservedCount =
            _opts.track_subset_selection_long_track_length_threshold;
        opts.imageCellSize = _opts.track_selection_image_grid_cell_size_pixels;
        opts.minTrackPerView = _opts.min_num_optimized_tracks_per_view;

        const auto goodTrackIds = SelectGoodTracks::Select(opts, *_scene);
        if (!goodTrackIds.empty()) {
            _scene->setTracksInViewsUnestimated(_scene->viewIds(),
                                                goodTrackIds);
            // TODO: use swap
            selectedTrackIds = goodTrackIds;
        }
    }
    else {
        // If the track selection fails or is not desired, then add all
        // tracks from the views we wish to optimize.
        std::set<TrackId> trackIds;
        for (const auto& viewId : selectedViewIds) {
            const auto* view = _scene->view(viewId);
            for (const auto& trackId : view->trackIds()) {
                trackIds.insert(trackId);
            }
        }
        selectedTrackIds = {trackIds.cbegin(), trackIds.cend()};
    }

    LOG(INFO) << "Selected " << selectedTrackIds.size()
              << " tracks to optimize.";

    // Perform partial BA.
    const auto ba_summary = BundleAdjustPartialScene(baOpts, selectedViewIds,
                                                     selectedTrackIds, _scene);

    removeOutlierTracks(selectedTrackIds,
                        _opts.max_reprojection_error_in_pixels);
    return ba_summary.success;
}

void IncrementalReconstruction::removeOutlierTracks(
    const std::vector<TrackId>& trackIds, double maxRpe)
{
    // Remove the outlier points based on the reprojection error and how
    // well-constrained the 3D points are.
    const auto numOutliers = _scene->setOutlierTracksUnestimated(
        trackIds, maxRpe, _opts.min_triangulation_angle_degrees);

    LOG(INFO) << numOutliers << " outlier points were removed.";
}

void IncrementalReconstruction::SetUnderconstrainedAsUnestimated()
{
    int num_underconstrained_views = -1;
    int num_underconstrained_tracks = -1;
    while (num_underconstrained_views != 0 &&
           num_underconstrained_tracks != 0) {
        num_underconstrained_views =
            _scene->setUnderconstrainedViewsUnestimated();
        num_underconstrained_tracks =
            _scene->setUnderconstrainedTracksUnestimated();
    }

    // If any views were removed then we need to update the localization
    // container so that we can try to re-estimate the view.
    if (num_underconstrained_views > 0) {
        const auto& viewIds = _viewGraph->viewIds();
        for (const auto& viewId : viewIds) {
            const auto* view = _scene->view(viewId);
            if (view && !view->estimated() &&
                !unlocalized_views_.contains(viewId)) {
                unlocalized_views_.insert(viewId);

                // Remove the view from the list of localized views.
                auto view_to_remove =
                    std::find(reconstructed_views_.begin(),
                              reconstructed_views_.end(), viewId);
                reconstructed_views_.erase(view_to_remove);
                --num_optimized_views_;
            }
        }
    }
}

} // namespace tl
