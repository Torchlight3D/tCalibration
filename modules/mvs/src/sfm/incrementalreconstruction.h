#pragma once

#include <tMvs/Types>

#include "reconstruction.h"

namespace tl {

class Scene;
class ViewGraph;

// Brief:
// Estimates the camera position and 3D structure of the scene using an
// incremental Structure from Motion approach. The method begins by first
// estimating the 3D structure and camera poses of 2 cameras based on their
// relative pose. Then additional cameras are added on sequentially and new 3D
// structure is estimated as new parts of the scene are observed. Bundle
// adjustment is repeatedly performed as more cameras are added to ensure high
// quality reconstructions and to avoid drift.
//
// The incremental SfM pipeline is as follows:
//   1. Choose an initial camera pair to reconstruct (if necessary).
//   2. Estimate 3D structure of the scene.
//   3. Bundle adjustment on the 2-view reconstruction.
//   4. Localize a new camera to the current 3D points. Choose the camera that
//   observes the most 3D points currently in the scene.
//   5. Estimate new 3D structure.
//   6. Bundle adjustment if the model has grown by more than 5% since the last
//   bundle adjustment.
//   7. Repeat steps 4-6 until all cameras have been added.
//
// Note that steps 1-3 are skipped if an "initialized" reconstruction (one that
// already contains estimated views and tracks) is passed into the Estimate
// function.
//
// Incremental SfM is generally considered to be more robust than global SfM
// methods; hwoever, it requires many more instances of bundle adjustment (which
// is very costly) and so incremental SfM is not as efficient or scalable.
class IncrementalReconstruction final : public Reconstruction
{
public:
    struct Options final : ReconstructionOptions
    {
        // If M is the maximum number of 3D points observed by any view, we want
        // to localize all views that observe > M *
        // multiple_view_localization_ratio 3D points. This allows for multiple
        // well-conditioned views to be added to the reconstruction before
        // needing bundle adjustment.
        double multiple_view_localization_ratio = 0.8;

        // When adding a new view to the current reconstruction, this is the
        // reprojection error that determines whether a 2D-3D correspondence is
        // an inlier during localization.
        //
        // NOTE: This threshold is with respect to an image that is 1024 pixels
        // wide. If the image dimensions are larger or smaller than this value
        // then the threshold will be appropriately scaled. This allows us to
        // use a single threshold for images that have varying resolutions.
        double absolute_pose_reprojection_error_threshold = 4.0;

        // Minimum number of inliers for absolute pose estimation to be
        // considered successful.
        int min_num_absolute_pose_inliers = 30;

        // Bundle adjustment of the entire reconstruction is triggered when the
        // reconstruction has grown by more than this percent. That is, if we
        // last ran BA when there were K views in the reconstruction and there
        // are now N views, then G = (N - K) / K is the percent that the model
        // has grown. We run bundle adjustment only if G is greater than this
        // variable. This variable is indicated in percent so e.g., 5.0 = 5%.
        double full_bundle_adjustment_growth_percent = 5.0;

        // During incremental SfM we run "partial" bundle adjustment on the most
        // recent views that have been added to the 3D reconstruction. This
        // parameter controls how many views should be part of the partial BA.
        int partial_bundle_adjustment_num_views = 20;

        // --------------- Reconstruction Localization Options ---------------

        // The PnP type that is used in the calibrated camera case
        PnPType localization_pnp_type = PnPType::DLS;

        // Ransac parameters
        double ransac_confidence = 0.9999;
        int ransac_min_iterations = 50;
        int ransac_max_iterations = 1000;
        bool ransac_use_mle = true;
        bool ransac_use_lo = true;
        int ransac_lo_start_iterations = 50;

        // Sets the ransac parameters from the reconstruction estimator options.
        // NOTE: This does not set the error threshold since that is application
        // specific. The caller must set this threshold.
        SacParameters toRansacParameters() const;
    };

    explicit IncrementalReconstruction(const Options& opts);

    Summary Estimate(ViewGraph* viewGraph, Scene* scene) override;

private:
    // Choose two cameras to use as the seed for incremental reconstruction.
    // These cameras should observe 3D points that are well-conditioned. We
    // determine the conditioning of 3D points by examining the median viewing
    // angle of the correspondences between the views.
    bool ChooseInitialViewPair();

    // The best initial view pairs for incremental SfM are the ones that have a
    // lot of matches but sufficient baseline between them. One measurement for
    // a well-constrained baseline between cameras is the number of inliers when
    // estimating a homography (more inliers means it is less constrained i.e.,
    // bad). This method chooses the view pairs with more than
    // min_num_verified_matches that have the fewest homography inliers.
    std::vector<ViewIdPair> OrderViewPairsByInitializationCriterion(
        int minVerifiedMatch) const;

    // Initialize the views based on the TwoViewInfo of the view pairs and set
    // the views as estimated.
    void initializeCamerasInViewPair(const ViewIdPair& id);

    // Estimates all possible 3D points in the view. This is useful during
    // incremental SfM because we only need to triangulate points that were
    // added with new views.
    void EstimateStructure(ViewId id);

    // The current percentage of cameras that have not been optimized by full
    // BA.
    double UnoptimizedGrowthPercentage() const;

    // Performs partial bundle adjustment on the model. Only the k most recent
    // cameras (and the tracks observed in those views) are optimized.
    bool partialBundleAdjustment();

    // Performs full bundle adjustment on the model.
    bool fullBundleAdjustment();

    // Chooses the next cameras to be localized according to which camera
    // observes the highest number of 3D points in the scene. This view is then
    // localized to using the calibrated or uncalibrated absolute pose
    // algorithm.
    void FindViewsToLocalize(std::vector<ViewId>* viewIds) const;

    // Remove any features that have too high of reprojection errors or are not
    // well-constrained. Only the input features are checked for outliers.
    void removeOutlierTracks(const std::vector<TrackId>& trackIds,
                             double maxRpe);

    // Set any views that do not observe enough 3D points to unestimated, and
    // similarly set and tracks that are not observed by enough views to
    // unestimated.
    void SetUnderconstrainedAsUnestimated();

private:
    Options _opts;
    Scene* _scene;
    ViewGraph* _viewGraph;

    // A container to keep track of which views need to be localized.
    std::unordered_set<ViewId> unlocalized_views_;

    // An *ordered* container to keep track of which views have been added to
    // the reconstruction. This is used to determine which views are optimized
    // during partial BA.
    std::vector<ViewId> reconstructed_views_;

    // Indicates the number of views that have been optimized with full BA.
    int num_optimized_views_;

    // DISALLOW_COPY_AND_ASSIGN(IncrementalReconstructionEstimator);
};

} // namespace tl
