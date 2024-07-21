#pragma once

#include <tCore/Global>

#include "reconstruction.h"

#include <tMvs/Poses/EstimateRotation>
#include <tMvs/Poses/EstimatePositionLiGT>
#include <tMvs/Poses/EstimatePositionLinear>
#include <tMvs/Poses/EstimatePositionLUD>
#include <tMvs/Poses/EstimatePositionNonlinear>

namespace tl {

// Brief:
// Estimates the camera poses and 3D structure of the scene using global method.
// Generally, the camera rotations are estimated globally first, then the
// positions are estimated using a global optimization.
//
// The pipeline for estimating camera poses and structure is as follows:
//   1. Filter potentially bad pairwise geometries by enforcing a loop constaint
//   on rotations that form a triplet.
//   2. Initialize focal lengths.
//   3. Estimate the global rotation for each camera.
//   4. Remove any pairwise geometries where the relative rotation is not
//   consistent with the global rotation.
//   5. Optimize the relative translation given the known rotations.
//   6. Filter potentially bad relative translations.
//   7. Estimate positions.
//   8. Estimate structure.
//   9. Bundle adjustment.
//   10. Retriangulate, and bundle adjust.
//
class GlobalReconstruction final : public Reconstruction
{
public:
    struct Options final : ReconstructionOptions
    {
        // ------------ More Bundle Adjustment Options -------------- //

        int num_retriangulation_iterations = 1;

        // ----------- Estimate Global Rotation Options ------------ //

        // Global rotation estimation algorithm to use
        GlobalRotationEstimatorType global_rotation_estimator_type =
            GlobalRotationEstimatorType::Robust;

        // After orientations are estimated, view pairs may be filtered/removed
        // if the relative rotation of the view pair differs from the relative
        // rotation formed by the global orientation estimations. Adjust this
        // threshold to control the threshold at which rotations are filtered.
        double rotation_filtering_max_difference_degrees = 5.;

        // --------------- Position Filtering Options --------------- //

        // Refine the relative translations based on the epipolar error and
        // known rotation estimations. This improve the quality of the
        // translation estimation.
        bool refine_relative_translations_after_rotation_estimation = true;

        // If true, the maximal rigid component of the viewing graph will be
        // extracted. This means that only the cameras that are well-constrained
        // for position estimation will be used. This method is somewhat slow,
        // so enabling it will cause a performance hit in terms of efficiency.
        //
        // NOTE: This method does not attempt to remove outlier 2-view
        // geometries, it only determines which cameras are well-conditioned for
        // position estimation.
        bool extract_maximal_rigid_subgraph = false;

        // If true, filter the pairwise translation estimates to remove
        // potentially bad relative poses. Removing potential outliers can
        // increase the performance of position estimation.
        bool filter_relative_translations_with_1dsfm = true;

        // Before the camera positions are estimated, it is wise to remove any
        // relative translations estimates that are low quality. See
        int translation_filtering_num_iterations = 48;
        double translation_filtering_projection_tolerance = 0.1;

        // Robust loss function scales for nonlinear estimation.
        double rotation_estimation_robust_loss_scale = 0.1;

        // Options of all different global position estimators.
        // FIXME: This pattern is not extendable

        // Global position estimation algorithm to use
        GlobalPositionEstimatorType global_position_estimator_type =
            GlobalPositionEstimatorType::LUD;

        NonlinearPositionEstimator::Options
            nonlinear_position_estimator_options;
        LinearPositionEstimator::Options
            linear_triplet_position_estimator_options;
        LiGTPositionEstimator::Options ligt_position_estimator_options;
        LeastUnsquaredDeviationPositionEstimator::Options
            least_unsquared_deviation_position_estimator_options;

        // For global SfM it may be advantageous to run a partial bundle
        // adjustment optimizing only the camera positions and 3d points while
        // holding camera orientation and intrinsics constant.
        bool refine_camera_positions_and_points_after_position_estimation =
            true;
    };

    explicit GlobalReconstruction(const Options& options);

    Summary Estimate(ViewGraph* viewGraph, Scene* scene) override;

private:
    bool filterInitialViewGraph();
    void initializeCameraIntrinsics();
    bool estimateGlobalRotations();
    void filterRotations();
    void optimizeRelativeTranslations();
    void filterRelativeTranslations();
    bool estimateGlobalPositions();
    void estimateStructure();
    bool bundleAdjustment();
    // The view orientations and camera intrinsics are held constant.
    bool bundleAdjustPositionsAndLandmarks();

private:
    DISABLE_COPY(GlobalReconstruction);

    Options _opts;
    // Cached data
    ViewGraph* _viewGraph; // Not owned, TODO: Could be shared
    Scene* _scene;         // Not owned, TODO: Could be shared
    std::unordered_map<ViewId, Eigen::Vector3d> _orientations;
    std::unordered_map<ViewId, Eigen::Vector3d> _positions;
};

} // namespace tl
