#pragma once

#include <thread>
#include <unordered_set>

#include <ceres/solver.h>

#include <tCamera/CameraIntrinsics>
#include <tMvs/Types>
#include <tSolver/LossFunctionCreator>

namespace tl {

struct BundleAdjustmentOptions
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

    // Use local parametrization for points. Apply increments in local tangent
    // space. Reduce from dim 4 -> 3
    bool use_homogeneous_local_point_parametrization = true;

    // if inverse depth parametrization should be used
    // if this is set to true use_homogeneous_point_parametrization is ignored
    bool use_inverse_depth_parametrization = false;

    // If true, the camera pose(orientations/positions) will be set to constant.
    // For instance, you have very accurate positions from GPS, while do not
    // know camera orientations. Then set camera position constant.
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

    // Inner iterations can improve the quality according to the Ceres issue.
    bool use_inner_iterations = false;

    // These variables may be useful to change if the optimization is converging
    // to a bad result.
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

struct BundleAdjustmentSummary
{
    // Cost change
    double initial_cost = 0.;
    double final_cost = 0.;

    // Setup time starts from configuration, ends before optimization.
    double setup_time_in_seconds = 0.;

    // Solve time start from optimization.
    double solve_time_in_seconds = 0.;

    // This only indicates whether the optimization was successfully run, and
    // makes no guarantees on the quality or convergence.
    bool success = false;
};

class Scene;

using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Bundle adjust all views and tracks in the scene.
BundleAdjustmentSummary BundleAdjustScene(
    const BundleAdjustmentOptions& options, Scene* scene);

// Bundle adjust the specified views and all tracks observed by those views.
BundleAdjustmentSummary BundleAdjustPartialScene(
    const BundleAdjustmentOptions& options,
    const std::unordered_set<ViewId>& viewIds,
    const std::unordered_set<TrackId>& trackIds, Scene* scene);

BundleAdjustmentSummary BundleAdjustPartialViews(
    const BundleAdjustmentOptions& options,
    const std::vector<ViewId>& mutableViewIds,
    const std::vector<ViewId>& constViewIds, Scene* scene);

// Bundle adjust the specified view.
BundleAdjustmentSummary BundleAdjustView(const BundleAdjustmentOptions& options,
                                         ViewId viewId, Scene* scene);

BundleAdjustmentSummary BundleAdjustView(const BundleAdjustmentOptions& options,
                                         ViewId viewId, Scene* scene,
                                         Matrix6d* covariance,
                                         double* variance);

// Bundle adjust specified views.
BundleAdjustmentSummary BundleAdjustViews(
    const BundleAdjustmentOptions& options, const std::vector<ViewId>& viewIds,
    Scene* scene);

BundleAdjustmentSummary BundleAdjustViews(
    const BundleAdjustmentOptions& options, const std::vector<ViewId>& viewIds,
    Scene* scene, std::map<ViewId, Matrix6d>* covariances, double* variance);

// Bundle adjust the specified track.
BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options, TrackId trackId, Scene* scene);

BundleAdjustmentSummary BundleAdjustTrack(
    const BundleAdjustmentOptions& options, TrackId trackId, Scene* scene,
    Eigen::Matrix3d* covariance, double* variance);

// Bundle adjust specified tracks.
BundleAdjustmentSummary BundleAdjustTracks(
    const BundleAdjustmentOptions& options,
    const std::vector<TrackId>& trackIds, Scene* scene);

BundleAdjustmentSummary BundleAdjustTracks(
    const BundleAdjustmentOptions& options,
    const std::vector<TrackId>& trackIds, Scene* scene,
    std::map<TrackId, Eigen::Matrix3d>* covariances, double* variance);

} // namespace tl
