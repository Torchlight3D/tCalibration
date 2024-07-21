#pragma once

#include <memory>
#include <string>

#include <tCamera/Camera>
#include <tMath/Ransac/RansacCreator>
#include <tMath/Solvers/LossFunction>
#include <tMvs/Types>
#include <tMvs/BA/BundleAdjustment>
#include <tMvs/Epipolar/Triangulation>

namespace tl {

class RandomNumberGenerator;
class Scene;
class ViewGraph;

// Global SfM methods are considered to be more scalable while incremental
// SfM is less scalable but often more robust.
// Reconstruction methods:
//     1. Global[default]:
//
//     2. Incremental:
//
//     3. Hybrid:
//
enum class ReconstructionType
{
    Global = 0,
    Incremental,
    Hybrid
};

// Track parametrization type:
//     0. Homo: 4D point --> optimize over homogeneous 4 vector
//     1. HomoManifold: 4D point --> optimize over ceres::SphereManifold<4>()
//     2. InverseDepth: optimize of track using inverse depth parametrization
enum class TrackParametrizationType
{
    Homo = 0,
    HomoManifold,
    InverseDepth
};

// The base Options of all the derived reconstruction pipelines. This Options
// consists of the options of the folloing processes:
//     1. Triangulation
//     2. Bundle adjustment
//     3. Landmark subsampling
//     4. Misc. Fitler thresholding, thread number, and etc.
struct ReconstructionOptions
{
    // The random number generator used to generate random numbers through
    // the reconstruction estimation process. If this is a nullptr then the
    // random generator will be initialized based on the current time.
    std::shared_ptr<RandomNumberGenerator> rng;

    // Number of threads to use.
    int num_threads = 1;

    // Minimum number of feature inliers between two views. Any two view
    // pairs (edges) in the view graph with fewer features than this value
    // will be removed in the initial view pairs filtering stage.
    int min_num_two_view_inliers = 30;

    // Maximum reprojection error. This is the threshold used for filtering
    // outliers after bundle adjustment.
    double max_reprojection_error_in_pixels = 5.;

    // --------------- Triangulation Options --------------- //

    // Minimum angle required between a 3D point and 2 viewing rays in order
    // to consider triangulation a success.
    double min_triangulation_angle_degrees = 3.;

    // The reprojection error to use for determining valid triangulation.
    double triangulation_max_reprojection_error_in_pixels = 5.;

    // Bundle adjust a track immediately after estimating it.
    bool bundle_adjust_tracks = true;

    // Bundle adjust a track immediately after estimating it.
    TriangulationMethodType triangulation_method =
        TriangulationMethodType::MIDPOINT;

    // --------------- Bundle Adjustment Options --------------- //

    // After computing a model and performing an initial BA, the
    // reconstruction can be further improved (and even densified) if we
    // attempt (again) to retriangulate any tracks that are currently
    // unestimated. For each retriangulation iteration we do the following:
    //   1. Remove features that are above max_reprojection_error_in_pixels.
    //   2. Triangulate all unestimated tracks.
    //   3. Perform full bundle adjustment.

    // The loss function type used in bundle adjustment. A robust loss function
    // greatly improves the BA robustness to outliers.
    LossFunctionType bundle_adjustment_loss_function_type =
        LossFunctionType::Trivial;

    // For robust loss functions, the robustness will begin for values that
    // have an error greater than this value. For example, Tukey loss will
    // have a constant loss when the error values are greater than this.
    double bundle_adjustment_robust_loss_width = 10.;

    // Use SPARSE_SCHUR for problems smaller than this size and
    // ITERATIVE_SCHUR for problems larger than this size.
    int min_cameras_for_iterative_solver = 1000;

    // If accurate calibration is known ahead of time then it is recommended
    // to set the camera intrinsics constant during bundle adjustment.
    // Othewise, you can choose which intrinsics to optimize.
    OptimizeIntrinsicsType intrinsics_to_optimize =
        OptimizeIntrinsicsType::FocalLength |
        OptimizeIntrinsicsType::RadialDistortion;

    // The track parametrizeation type to use in bundle adjustment.
    // Standard is XYZW which will optimize a homogeneous vector, but on a
    // euclidean manifold.
    TrackParametrizationType track_parametrization_type =
        TrackParametrizationType::HomoManifold;

    // --------------- Track Subsampling Options --------------- //

    // Bundle adjustment performs joint nonlinear optimization of point
    // positions and camera poses by minimizing reprojection error. For many
    // scenes, the 3d points can be highly redundant such that adding more
    // points only marginally improves the reconstruction quality (if at
    // all) despite a large increase in runtime. As such, we can reduce the
    // number of 3d points used in bundle adjustment and still achieve
    // similar or even better quality reconstructions by carefully choosing
    // the points such that they properly constrain the optimization.
    //
    // If subsampling the tracks is set to true, then the 3d points are
    // chosen such that they fit the following criteria:
    //
    //    a. High confidence (i.e. low reprojection error).
    //    b. Long tracks are preferred.
    //    c. The tracks used for optimization provide a good spatial coverage in
    //    each image.
    //    d. Each view observes at least K optimized tracks.

    // Tracks are selected to optimize for these criteria using the
    // thresholds below.
    bool subsample_tracks_for_bundle_adjustment = false;

    // While long tracks are preferred during the track subsampling, it's also
    // worth noticing that long tracks often are more likely to contain
    // outliers. Thus, we cap the track length for track selection at 10 then
    // sort tracks first by the truncated track length, then secondarily by
    // their mean reprojection error. This allows us to choose the high quality
    // tracks among all the long tracks.
    int track_subset_selection_long_track_length_threshold = 10;

    // To find the coverage condition of detected tracks, we divide each image
    // into an image grid with specific grid cell widths. The top ranked track
    // in each grid cell is chosen to be optimized so that each image has a good
    // spatial coverage.
    int track_selection_image_grid_cell_size_pixels = 100;

    // The minimum number of optimized tracks required for each view when
    // using track subsampling. If the view does not observe this many
    // tracks, then all tracks in the view are optimized.
    int min_num_optimized_tracks_per_view = 200;

    BundleAdjustment::Options toBundleAdjustmentOptions(int numViews) const;
};

// Brief:
// The base class of top level reconstruction estimator class, which can should
// build a reconstruction from a view graph and an unestimated scene with the
// corresponding views of the view graph. The global camera poses and 3D
// landmark positions are estimated as the main outputs.

class Reconstruction
{
public:
    virtual ~Reconstruction() = default;

    struct Summary
    {
        bool success = false;
        std::vector<ViewId> estimated_views;
        std::vector<TrackId> estimated_tracks;

        // All times are given in seconds.
        double camera_intrinsics_calibration_time = 0.;
        double pose_estimation_time = 0.;
        double triangulation_time = 0.;
        double bundle_adjustment_time = 0.;
        double total_time = 0.;

        std::string message;
    };

    // Estimates the camera poses for a reconstruction given the view graph
    // describing the multi-view correspondences.
    virtual Summary Estimate(ViewGraph* viewGraph, Scene* scene) = 0;

    // static Reconstruction* Create(const Options& options);

protected:
};

// Set the camera intrinsics for every views in the scene from the
// CameraMetaData.
//     1. If no focal prior is provided, the focal length will be set to a value
//     corresponding to a median viewing angle.
//     2. If no principal point is provided, the principal point will be simply
//     set to half of the corresponding image size dimension.
//     3. If no image size is provided, nothing happen, it's INVALID.
void setCameraIntrinsicsFromMetaData(Scene* scene);

} // namespace tl
