#pragma once

#include <thread>

#include <ceres/types.h>

#include <tMath/Solvers/LossFunction>
#include <tMvs/Scene>

namespace tl {

using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Brief:
// This class sets up nonlinear optimization problems for bundle adjustment.
//
// Explanation:
// Bundle adjustment problems are set up by adding views and tracks to be
// optimized. Only the views and tracks supplied with addView and addTrack
// will be optimized. All other parameters are held constant.
//
// Note:
// It is required that AddViews is called before AddTracks if any views are
// being optimized.
class BundleAdjustment
{
public:
    struct Options
    {
        // The loss function type used in BA, default standard L2 loss function
        LossFunctionType loss_function_type = LossFunctionType::Trivial;

        double robust_loss_width = 2.;
        // e.g. 1cm for downweighting depth priors as outliers
        double robust_loss_width_depth_prior = 0.01;

        // For larger problems (> 1000 cameras) it is recommended to use the
        // ITERATIVE_SCHUR solver.
        ceres::LinearSolverType linear_solver_type = ceres::SPARSE_SCHUR;
        ceres::PreconditionerType preconditioner_type = ceres::SCHUR_JACOBI;
        ceres::VisibilityClusteringType visibility_clustering_type =
            ceres::CANONICAL_VIEWS;

        // This verbose will pass to ceres verbose
        bool verbose = false;

        // Use local parametrization for points. Apply increments in local
        // tangent space. Reduce from dim 4 -> 3
        bool use_homogeneous_local_point_parametrization = true;

        // if inverse depth parametrization should be used
        // if this is set to true use_homogeneous_point_parametrization is
        // ignored
        bool use_inverse_depth_parametrization = false;

        // If true, the camera pose(orientations/positions) will be set to
        // constant. For instance, you have very accurate positions from GPS,
        // while do not know camera orientations. Then set camera position
        // constant.
        bool constant_camera_orientation = false;
        bool constant_camera_position = false;

        // Indicates which intrinsic parameters should be optimized during BA.
        // By default, we don't optimize intrinsics.
        OptimizeIntrinsicsType intrinsics_to_optimize =
            OptimizeIntrinsicsType::None;

        int num_threads = std::thread::hardware_concurrency();
        int max_num_iterations = 100;

        // Max BA time is 1 hour.
        double max_solver_time_in_seconds = 3600.;

        // Inner iterations can improve the quality according to the Ceres
        // issue.
        bool use_inner_iterations = false;

        // These variables may be useful to change if the optimization is
        // converging to a bad result.
        double function_tolerance = 1e-6;
        double gradient_tolerance = 1e-10;
        double parameter_tolerance = 1e-8;
        double max_trust_region_radius = 1e12;

        // Use position priors
        bool use_position_priors = false;

        // Add depth priors
        bool use_depth_priors = false;

        // Adjusting orthographic camera
        bool orthographic_camera = false;

        // Use gravity prior
        bool use_gravity_prior = false;
    };

    // The scene maybe be modified during bundle adjustment.
    BundleAdjustment(const Options& options, Scene* scene);
    ~BundleAdjustment();

    /// Data
    void addView(ViewId id);
    void addTrack(TrackId id);
    void setFixedView(ViewId id);

    /// Actions
    struct Summary
    {
        // Cost change
        double initial_cost = 0.;
        double final_cost = 0.;

        // Setup time starts from configuration, ends before optimization.
        double setup_time_in_seconds = 0.;

        // Solve time start from optimization.
        double solve_time_in_seconds = 0.;

        // This only indicates whether the optimization was successfully run,
        // and makes no guarantees on the quality or convergence.
        bool success = false;
    };
    Summary optimize();

    /// Results
    bool calcCovarianceForTrack(TrackId id, Eigen::Matrix3d* covariance) const;
    bool calcCovarianceForTracks(
        const std::vector<TrackId>& ids,
        std::map<TrackId, Eigen::Matrix3d>* covariances) const;

    bool calcCovarianceForView(ViewId id, Matrix6d* covariance) const;
    bool calcCovarianceForViews(const std::vector<ViewId>& ids,
                                std::map<ViewId, Matrix6d>* covariances) const;
    bool calcCovarianceForCamera(CameraId id,
                                 Eigen::MatrixXd* covariance) const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend class Impl;
};

////////////// Helper functions ////////////////////

// Bundle adjust all views and tracks in the scene.
BundleAdjustment::Summary BundleAdjustScene(
    const BundleAdjustment::Options& options, Scene* scene);

// Bundle adjust the specified views and all tracks observed by those views.
BundleAdjustment::Summary BundleAdjustPartialScene(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& viewIds, const std::vector<TrackId>& trackIds,
    Scene* scene);

BundleAdjustment::Summary BundleAdjustPartialViewsConstant(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& mutableViewIds,
    const std::vector<ViewId>& constViewIds, Scene* scene);

// Bundle adjust the specified view.
BundleAdjustment::Summary BundleAdjustView(
    const BundleAdjustment::Options& options, ViewId viewId, Scene* scene);

BundleAdjustment::Summary BundleAdjustView(
    const BundleAdjustment::Options& options, ViewId viewId, Scene* scene,
    Matrix6d* covariance, double* variance);

// Bundle adjust specified views.
BundleAdjustment::Summary BundleAdjustViews(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& viewIds, Scene* scene);

BundleAdjustment::Summary BundleAdjustViews(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& viewIds, Scene* scene,
    std::map<ViewId, Matrix6d>* covariances, double* variance);

BundleAdjustment::Summary BundleAdjustViews(
    const BundleAdjustment::Options& options,
    const std::vector<ViewId>& viewIds, Scene* scene, CameraId camId,
    Eigen::MatrixXd* covariance, double* variance);

// Bundle adjust the specified track.
BundleAdjustment::Summary BundleAdjustTrack(
    const BundleAdjustment::Options& options, TrackId trackId, Scene* scene);

BundleAdjustment::Summary BundleAdjustTrack(
    const BundleAdjustment::Options& options, TrackId trackId, Scene* scene,
    Eigen::Matrix3d* covariance, double* variance);

// Bundle adjust specified tracks.
BundleAdjustment::Summary BundleAdjustTracks(
    const BundleAdjustment::Options& options,
    const std::vector<TrackId>& trackIds, Scene* scene);

BundleAdjustment::Summary BundleAdjustTracks(
    const BundleAdjustment::Options& options,
    const std::vector<TrackId>& trackIds, Scene* scene,
    std::map<TrackId, Eigen::Matrix3d>* covariances, double* variance);

struct TwoViewBundleAdjustmentOptions
{
    BundleAdjustment::Options ba_options;
    bool constant_camera1_intrinsics{true};
    bool constant_camera2_intrinsics{true};
};

class Camera;
struct Feature2D2D;
struct ViewPairInfo;

// Performs bundle adjustment on the two views assuming that both views observe
// all of the 3D points. The cameras should be initialized with intrinsics and
// extrinsics appropriately, and the 3D points should be set (e.g., from
// triangulation) before calling this method. The first camera pose is held
// constant during BA, and the optimized pose of the second camera, (optionally)
// intrinsics, and 3D points are returned. The indices of the feature
// correspondences should match the 3D point indices.
BundleAdjustment::Summary BundleAdjustTwoViews(
    const TwoViewBundleAdjustmentOptions& options,
    const std::vector<Feature2D2D>& correspondences, Camera* camera1,
    Camera* camera2, std::vector<Eigen::Vector4d>* points3d);

// Brief:
// Performs bundle adjustment to find the optimal rotation and translation
// describing the two views. This is done without the need for 3D points.
//
// Ref:
// "Exact Two-Image Structure from Motion" by John Oliensis (PAMI2002)
//
// NOTE: The 2D-2D correspondences must be normalized by the focal length and
// principal point.
BundleAdjustment::Summary BundleAdjustTwoViewsAngular(
    const BundleAdjustment::Options& options,
    const std::vector<Feature2D2D>& correspondences, ViewPairInfo* info);

// Brief:
// Optimizes a fundamentral matrix by its manifold form.
//
// Ref:
// "Non-Linear Estimation of the Fundamental Matrix With Minimal Parameters" by
// Bartoli and Sturm (PAMI 2004)
BundleAdjustment::Summary OptimizeFundamental(
    const BundleAdjustment::Options& options,
    const std::vector<Feature2D2D>& correspondences, Eigen::Matrix3d* fmatrix);

// Brief:
// Optimizes a homography matrix by its manifold form.
//
// Ref:
// ceres-solver/examples/libmv_homography.cc
BundleAdjustment::Summary OptimizeHomography(
    const BundleAdjustment::Options& options,
    const std::vector<Feature2D2D>& correspondences,
    Eigen::Matrix3d* homography);

} // namespace tl
