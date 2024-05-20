#include "bundle_adjustment_solver.h"

#include <ceres/rotation.h>

#include <tCamera/CameraIntrinsics>
#include <tCamera/DoubleSphereCameraModel>
#include <tCamera/ExtendedUnifiedCameraModel>
#include <tCamera/OmnidirectionalCameraModel>
#include <tCamera/ReprojectionError>
#include <tCore/ContainerUtils>
#include <tMath/Solvers/LossFunction>

#include "../residuals/inversereprojectionerror.h"

namespace tl {

namespace {

void baOptionsToCeresOptions(const BundleAdjustmentOptions& ba_options,
                             ceres::Solver::Options* ceres_options)
{
    ceres_options->linear_solver_type = ba_options.linear_solver_type;
    ceres_options->preconditioner_type = ba_options.preconditioner_type;
    ceres_options->visibility_clustering_type =
        ba_options.visibility_clustering_type;
    ceres_options->logging_type =
        ba_options.verbose ? ceres::PER_MINIMIZER_ITERATION : ceres::SILENT;
    ceres_options->num_threads = ba_options.num_threads;
    ceres_options->max_num_iterations = ba_options.max_num_iterations;
    ceres_options->max_solver_time_in_seconds =
        ba_options.max_solver_time_in_seconds;
    ceres_options->use_inner_iterations = ba_options.use_inner_iterations;
    ceres_options->function_tolerance = ba_options.function_tolerance;
    ceres_options->gradient_tolerance = ba_options.gradient_tolerance;
    ceres_options->parameter_tolerance = ba_options.parameter_tolerance;
    ceres_options->max_trust_region_radius = ba_options.max_trust_region_radius;

    // Solver options takes ownership of the ordering so that we can order the
    // BA problem by points and cameras.
    ceres_options->linear_solver_ordering.reset(
        new ceres::ParameterBlockOrdering);
}

} // namespace

struct DepthPriorError
{
    const Feature feature_;

    explicit DepthPriorError(const Feature& feature) : feature_(feature) {}

    template <typename T>
    bool operator()(const T* const extrinsics, const T* const point,
                    T* residual) const
    {
        using Vector3 = Eigen::Map<const Eigen::Vector3<T>>;

        // Remove the translation.
        Eigen::Vector3<T> adjusted_point =
            Vector3(point) - point[3] * Vector3(extrinsics + Camera::Position);

        // If the point is too close to the camera center then the point cannot
        // be constrained by triangulation. This is likely to only occur when a
        // 3d point is seen by 2 views and the camera center of 1 view lies on
        // or neare the optical axis of the other view.
        //
        // Since we do not know the camera model we cannot say that the point
        // must be in front of the camera (e.g., wide angle cameras that have >
        // 180 degree FOV). Instead we simply force that the point is not near
        // the camera center.
        static const T kVerySmallNumber(1e-8);
        if (adjusted_point.squaredNorm() < kVerySmallNumber) {
            return false;
        }

        // Rotate the point to obtain the point in the camera coordinate system.
        T rotated_point[3];
        ceres::AngleAxisRotatePoint(extrinsics + Camera::Orientation,
                                    adjusted_point.data(), rotated_point);

        const T sqrt_information =
            T(1. / ceres::sqrt(feature_.depth_prior_variance()));

        (*residual) =
            sqrt_information * (rotated_point[2] - T(feature_.depth_prior()));

        return true;
    }

    static ceres::CostFunction* create(const Feature& feature)
    {
        constexpr int kResidualSize{1};
        constexpr int kPointSize{4};
        return new ceres::AutoDiffCostFunction<
            DepthPriorError, kResidualSize, Camera::ExtrinsicsSize, kPointSize>(
            new DepthPriorError(feature));
    }
};

struct PositionError
{
    const Eigen::Vector3d position_prior_;
    const Eigen::Matrix3d position_prior_sqrt_information_;

    PositionError(const Eigen::Vector3d& position_prior,
                  const Eigen::Matrix3d& position_prior_sqrt_information)
        : position_prior_(position_prior),
          position_prior_sqrt_information_(position_prior_sqrt_information)
    {
    }

    template <typename T>
    bool operator()(const T* const extrinsics, T* residual) const
    {
        Eigen::Map<Eigen::Vector3<T>> res(residual);
        Eigen::Map<const Eigen::Vector3<T>> position(extrinsics +
                                                     Camera::Position);

        res = position_prior_sqrt_information_.cast<T>() *
              (position_prior_.cast<T>() - position);
        return true;
    }

    static ceres::CostFunction* create(
        const Eigen::Vector3d& position_prior,
        const Eigen::Matrix3d& position_prior_sqrt_information)
    {
        constexpr int kResidualSize{3};
        return new ceres::AutoDiffCostFunction<PositionError, kResidualSize,
                                               Camera::ExtrinsicsSize>(
            new PositionError(position_prior, position_prior_sqrt_information));
    }
};

void BundleAdjuster::setCameraExtrinsicsParameterization()
{
    if (options_.constant_camera_orientation &&
        options_.constant_camera_position) {
        for (const auto& viewId : m_optimizedViewIds) {
            setCameraExtrinsicsConstant(viewId);
        }
    }
    else if (options_.constant_camera_orientation) {
        for (const auto& viewId : m_optimizedViewIds) {
            setCameraOrientationConstant(viewId);
        }
    }
    else if (options_.constant_camera_position) {
        for (const auto& viewId : m_optimizedViewIds) {
            setCameraPositionConstant(viewId);
        }
    }

    // For orthographic cameras we set tz = 0
    if (options_.orthographic_camera) {
        for (const auto& viewId : m_optimizedViewIds) {
            setTzConstant(viewId);
        }
    }
}

void BundleAdjuster::setCameraIntrinsicsParameterization()
{
    for (const auto& camId : m_optimizedCameraIds) {
        auto intrinsics = intrinsicsOfCamera(camId);
        auto* parameters = intrinsics->rParameters();

        // Set parameter bound
        // 1. Focal length
        problem_->SetParameterLowerBound(parameters, CameraIntrinsics::Fx, 1.);

        // 2. Distortion
        switch (intrinsics->type()) {
            case CameraIntrinsics::Type::DoubleSphere: {
                problem_->SetParameterLowerBound(
                    parameters, DoubleSphereCameraModel::Xi, -1.);
                problem_->SetParameterUpperBound(
                    parameters, DoubleSphereCameraModel::Xi, 1.);
                problem_->SetParameterLowerBound(
                    parameters, DoubleSphereCameraModel::Alpha, 0.);
                problem_->SetParameterUpperBound(
                    parameters, DoubleSphereCameraModel::Alpha, 1.);
            } break;
            case CameraIntrinsics::Type::ExtendedUnified: {
                problem_->SetParameterLowerBound(
                    parameters, ExtendedUnifiedCameraModel::Alpha, 0.);
                problem_->SetParameterUpperBound(
                    parameters, ExtendedUnifiedCameraModel::Alpha, 1.);
                problem_->SetParameterLowerBound(
                    parameters, ExtendedUnifiedCameraModel::Beta, 0.1);
            } break;
            case CameraIntrinsics::Type::Omnidirectional: {
                // Do nothing now
            } break;
            default:
                break;
        }

        // Set the constant parameters if any are requested.
        const std::vector<int> const_indices =
            intrinsics->constantParameterIndices(
                options_.intrinsics_to_optimize);

        if (const_indices.size() == intrinsics->numParameters()) {
            problem_->SetParameterBlockConstant(parameters);
        }
        else if (const_indices.size() > 0) {
            auto* manifold = new ceres::SubsetManifold(
                intrinsics->numParameters(), const_indices);
            problem_->SetManifold(parameters, manifold);
        }
    }

    // Set camera intrinsics to be constant if no cameras in the intrinsics
    // group are being optimized.
    for (const auto& camId : m_potentialConstantCameraIds) {
        if (m_optimizedCameraIds.contains(camId)) {
            continue;
        }

        auto camera_intrinsics = intrinsicsOfCamera(camId);
        problem_->SetParameterBlockConstant(camera_intrinsics->rParameters());
    }
}

CameraIntrinsics::Ptr BundleAdjuster::intrinsicsOfCamera(CameraId camId)
{
    const auto sharedCameraViewIds = scene_->sharedCameraViewIds(camId);
    CHECK(!sharedCameraViewIds.empty());

    // Use first viewId as representative
    const auto viewId = *sharedCameraViewIds.begin();
    return scene_->rView(viewId)->rCamera().cameraIntrinsics();
}

void BundleAdjuster::setCameraExtrinsicsConstant(ViewId viewId)
{
    auto* view = scene_->rView(viewId);
    auto& camera = view->rCamera();
    problem_->SetParameterBlockConstant(camera.rExtrinsics());
}

void BundleAdjuster::setCameraPositionConstant(ViewId viewId)
{
    const std::vector parameters{Camera::Position + 0, Camera::Position + 1,
                                 Camera::Position + 2};
    auto* manifold =
        new ceres::SubsetManifold(Camera::ExtrinsicsSize, parameters);
    auto* view = scene_->rView(viewId);
    auto& camera = view->rCamera();
    problem_->SetManifold(camera.rExtrinsics(), manifold);
}

void BundleAdjuster::setCameraOrientationConstant(ViewId viewId)
{
    const std::vector parameters{Camera::Orientation + 0,
                                 Camera::Orientation + 1,
                                 Camera::Orientation + 2};
    auto* manifold =
        new ceres::SubsetManifold(Camera::ExtrinsicsSize, parameters);
    auto* view = scene_->rView(viewId);
    auto& camera = view->rCamera();
    problem_->SetManifold(camera.rExtrinsics(), manifold);
}

void BundleAdjuster::setTzConstant(ViewId viewId)
{
    const std::vector parameters{Camera::Position + 2};
    auto* manifold =
        new ceres::SubsetManifold(Camera::ExtrinsicsSize, parameters);
    auto* view = scene_->rView(viewId);
    auto& camera = view->rCamera();
    problem_->SetManifold(camera.rExtrinsics(), manifold);
}

void BundleAdjuster::setTrackConstant(TrackId trackId)
{
    const auto* track = scene_->track(trackId);
    if (options_.use_inverse_depth_parametrization) {
        problem_->SetParameterBlockConstant(track->inverseDepth());
    }
    else {
        problem_->SetParameterBlockConstant(track->position().data());
    }
}

void BundleAdjuster::setTrackVariable(TrackId trackId)
{
    auto* track = scene_->rTrack(trackId);
    if (options_.use_inverse_depth_parametrization) {
        problem_->SetParameterBlockVariable(track->rInverseDepth());
    }
    else {
        problem_->SetParameterBlockVariable(track->rPosition().data());
    }
}

void BundleAdjuster::setHomogeneousPointParametrization(TrackId trackId)
{
    auto* track = scene_->rTrack(trackId);
    auto* pointManifold = new ceres::SphereManifold<4>();
    problem_->SetManifold(track->rPosition().data(), pointManifold);
}

void BundleAdjuster::setCameraSchurGroups(ViewId viewId)
{
    constexpr int kIntrinsicsParameterGroup{1};
    constexpr int kExtrinsicsParameterGroup{2};

    auto* view = scene_->rView(viewId);
    auto& camera = view->rCamera();

    // Add camera parameters to groups 1 and 2.
    // The extrinsics *must* belong to group 2. This is because inner
    // iterations uses a reverse ordering of elimination and the Schur-based
    // solvers require the first group to be an independent set. Since the
    // intrinsics may be shared, they are not guaranteed to form an
    // independent set and so we must use the extrinsics in group 2.
    parameter_ordering_->AddElementToGroup(camera.rExtrinsics(),
                                           kExtrinsicsParameterGroup);
    parameter_ordering_->AddElementToGroup(camera.rIntrinsics(),
                                           kIntrinsicsParameterGroup);
}

void BundleAdjuster::setTrackSchurGroup(TrackId trackId)
{
    constexpr int kTrackParameterGroup{0};

    auto* track = scene_->rTrack(trackId);
    // Set the parameter ordering for Schur elimination. We do this after
    // the loop above so that the track is already added to the problem.
    if (options_.use_inverse_depth_parametrization) {
        parameter_ordering_->AddElementToGroup(track->rInverseDepth(),
                                               kTrackParameterGroup);
    }
    else {
        parameter_ordering_->AddElementToGroup(track->rPosition().data(),
                                               kTrackParameterGroup);
    }
}

void BundleAdjuster::addReprojectionErrorResidual(const Feature& feature,
                                                  Camera* camera, Track* track)
{
    // Add the residual for the track to the problem. The shared intrinsics
    // parameter block will be set to constant after the loop if no
    // optimized cameras share the same camera intrinsics.
    problem_->AddResidualBlock(
        createReprojectionErrorCostFunction(camera->cameraIntrinsicsModel(),
                                            feature.point_),
        loss_function_.get(), camera->rExtrinsics(), camera->rIntrinsics(),
        track->rPosition().data());
}

void BundleAdjuster::addInvReprojectionErrorResidual(
    const Feature& feature, const Eigen::Vector3d& ref_bearing,
    Camera* camera_ref, Camera* camera_other, Track* track)
{
    // Add the residual for the track to the problem. The shared intrinsics
    // parameter block will be set to constant after the loop if no
    // optimized cameras share the same camera intrinsics.
    if (camera_ref == camera_other) {
        problem_->AddResidualBlock(
            createInvReprojectionPoseErrorCostFunction(
                camera_ref->cameraIntrinsicsModel(), feature, ref_bearing),
            loss_function_.get(), camera_ref->rExtrinsics(),
            camera_ref->rIntrinsics(), track->rInverseDepth());
    }
    else {
        problem_->AddResidualBlock(
            createInvReprojectionPoseErrorCostFunction(
                camera_other->cameraIntrinsicsModel(), feature, ref_bearing),
            loss_function_.get(), camera_ref->rExtrinsics(),
            camera_other->rExtrinsics(), camera_other->rIntrinsics(),
            track->rInverseDepth());
    }
}

void BundleAdjuster::addPositionPriorErrorResidual(View* view, Camera* camera)
{
    // Adds a position priors to the camera poses. This can for example be a
    // GPS position.
    problem_->AddResidualBlock(
        PositionError::create(view->positionPrior(), view->positionPriorSqrt()),
        nullptr, camera->rExtrinsics());
}

void BundleAdjuster::addDepthPriorErrorResidual(const Feature& feature,
                                                Camera* camera, Track* track)
{
    problem_->AddResidualBlock(
        DepthPriorError::create(feature), depth_prior_loss_function_.get(),
        camera->rExtrinsics(), track->rPosition().data());
}

///------- BundleAdjuster starts from here
BundleAdjuster::BundleAdjuster(const BundleAdjustmentOptions& options,
                               Scene* scene)
    : options_(options), scene_(scene)
{
    CHECK_NOTNULL(scene);

    loss_function_ = createLossFunction(options.loss_function_type,
                                        options.robust_loss_width);
    depth_prior_loss_function_ = createLossFunction(
        options.loss_function_type, options.robust_loss_width_depth_prior);

    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_.reset(new ceres::Problem(problem_options));

    // Set solver options.
    baOptionsToCeresOptions(options, &solver_options_);
    parameter_ordering_ = solver_options_.linear_solver_ordering.get();
}

void BundleAdjuster::addView(ViewId viewId)
{
    auto* view = CHECK_NOTNULL(scene_->rView(viewId));
    if (!view->estimated() || m_optimizedViewIds.contains(viewId)) {
        return;
    }

    m_optimizedViewIds.emplace(viewId);

    setCameraSchurGroups(viewId);

    m_optimizedCameraIds.emplace(scene_->cameraId(viewId));

    // Add residuals for all tracks in the view.
    auto& camera = view->rCamera();
    for (const auto& trackId : view->trackIds()) {
        const auto* feature = CHECK_NOTNULL(view->featureOf(trackId));
        auto* track = CHECK_NOTNULL(scene_->rTrack(trackId));
        // Only consider tracks with an estimated 3d point.
        if (!track->estimated()) {
            continue;
        }

        // Add the reprojection error to the optimization.
        addReprojectionErrorResidual(*feature, &camera, track);

        // Add the point to group 0.
        setTrackConstant(trackId);

        // Add depth priors if available
        if (options_.use_depth_priors && feature->depth_prior() != 0.) {
            addDepthPriorErrorResidual(*feature, &camera, track);
        }
    }

    // Add a position prior if available
    if (options_.use_position_priors && view->hasPositionPrior()) {
        addPositionPriorErrorResidual(view, &camera);
    }
}

void BundleAdjuster::addTrack(TrackId trackId)
{
    auto* track = CHECK_NOTNULL(scene_->rTrack(trackId));
    if (!track->estimated() || m_optimizedTrackIds.contains(trackId)) {
        return;
    }

    m_optimizedTrackIds.emplace(trackId);

    // Add all observations of the track to the problem.
    const auto& observedViewIds = track->viewIds();
    for (const auto& viewId : observedViewIds) {
        auto* view = CHECK_NOTNULL(scene_->rView(viewId));
        if (m_optimizedViewIds.contains(viewId) || !view->estimated()) {
            continue;
        }

        const auto* feature = CHECK_NOTNULL(view->featureOf(trackId));
        auto& camera = view->rCamera();

        addReprojectionErrorResidual(*feature, &camera, track);

        // Any camera that reaches this point was not added by AddView() and
        // so we want to mark it as constant.
        setCameraExtrinsicsConstant(viewId);

        // Mark the camera intrinsics as "potentially constant." We only set
        // the parameter block to constant if the shared intrinsics are not
        // shared with cameras that are being optimized.
        const auto camId = scene_->cameraId(viewId);
        m_potentialConstantCameraIds.emplace(camId);
    }

    setTrackVariable(trackId);
    setTrackSchurGroup(trackId);

    if (options_.use_homogeneous_local_point_parametrization) {
        setHomogeneousPointParametrization(trackId);
    }
}

BundleAdjustmentSummary BundleAdjuster::optimize()
{
    // Set extrinsics parameterization of the camera poses. This will set
    // orientation and/or positions as constant if desired.
    setCameraExtrinsicsParameterization();

    // Set intrinsics group parameterization. This will control which of the
    // intrinsics parameters or optimized or held constant. Note that each
    // camera intrinsics group may be a different camera and/or a different
    // camera intrinsics model.
    setCameraIntrinsicsParameterization();

    // NOTE: Use the reverse BA order (i.e., using cameras then points) is a
    // good idea for inner iterations.
    if (solver_options_.use_inner_iterations &&
        !options_.use_inverse_depth_parametrization) {
        solver_options_.inner_iteration_ordering.reset(
            new ceres::ParameterBlockOrdering(*parameter_ordering_));
        solver_options_.inner_iteration_ordering->Reverse();
    }

    ceres::Solver::Summary ceres_summary;
    ceres::Solve(solver_options_, problem_.get(), &ceres_summary);
    LOG_IF(INFO, options_.verbose) << ceres_summary.FullReport();

    BundleAdjustmentSummary ba_summary;
    ba_summary.setup_time_in_seconds =
        ceres_summary.preprocessor_time_in_seconds;
    ba_summary.solve_time_in_seconds = ceres_summary.total_time_in_seconds;
    ba_summary.initial_cost = ceres_summary.initial_cost;
    ba_summary.final_cost = ceres_summary.final_cost;

    // This only indicates whether the optimization was successfully run and
    // makes no guarantees on the quality or convergence.
    ba_summary.success = ceres_summary.IsSolutionUsable();

    return ba_summary;
}

bool BundleAdjuster::calcCovarianceForTrack(TrackId trackId,
                                            Eigen::Matrix3d* covariance)
{
    const auto* track = scene_->track(trackId);
    *covariance = Eigen::Matrix3d::Identity();

    const double* position = track->position().data();

    if (problem_->IsParameterBlockConstant(position) ||
        !problem_->HasParameterBlock(position)) {
        return false;
    }

    ceres::Covariance covariance_estimator{covariance_options_};
    std::vector<std::pair<const double*, const double*>> covariance_blocks = {
        std::make_pair(position, position)};

    if (!covariance_estimator.Compute(covariance_blocks, problem_.get())) {
        return false;
    }

    covariance_estimator.GetCovarianceMatrixInTangentSpace(
        {position}, (*covariance).data());
    return true;
}

bool BundleAdjuster::calcCovarianceForTracks(
    const std::vector<TrackId>& trackIds,
    std::map<TrackId, Eigen::Matrix3d>* covariances)
{
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    std::vector<TrackId> estTrackIds;
    for (const auto& trackId : trackIds) {
        const auto* track = scene_->track(trackId);
        const double* position = track->position().data();
        if (problem_->IsParameterBlockConstant(position) ||
            !problem_->HasParameterBlock(position)) {
            LOG(ERROR) << "There was a track that could not be found in the "
                          "reconstruction or is set to fixed! "
                          "No covariance estimation possible.";
            return false;
        }

        estTrackIds.push_back(trackId);
        covariance_blocks.push_back(std::make_pair(position, position));
    }

    ceres::Covariance covariance_estimator{covariance_options_};
    if (!covariance_estimator.Compute(covariance_blocks, problem_.get())) {
        return false;
    }

    for (size_t i{0}; i < estTrackIds.size(); ++i) {
        Eigen::Matrix3d cov;
        covariance_estimator.GetCovarianceMatrixInTangentSpace(
            {covariance_blocks[i].first}, cov.data());
        covariances->insert(std::make_pair(estTrackIds[i], cov));
    }

    return true;
}

bool BundleAdjuster::calcCovarianceForView(ViewId viewId,
                                           Matrix6d* covariance) const
{
    *covariance = Matrix6d::Identity();

    const double* extrinsics = scene_->view(viewId)->camera().extrinsics();
    if (problem_->IsParameterBlockConstant(extrinsics) ||
        !problem_->HasParameterBlock(extrinsics)) {
        return false;
    }

    ceres::Covariance covariance_estimator(covariance_options_);
    std::vector<std::pair<const double*, const double*>> covariance_blocks = {
        std::make_pair(extrinsics, extrinsics)};
    if (!covariance_estimator.Compute(covariance_blocks, problem_.get())) {
        return false;
    }

    covariance_estimator.GetCovarianceMatrixInTangentSpace(
        {extrinsics}, (*covariance).data());
    return true;
}

bool BundleAdjuster::calcCovarianceForViews(
    const std::vector<ViewId>& viewIds,
    std::map<ViewId, Matrix6d>& covariances) const
{
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    std::vector<ViewId> estViewIds;
    for (const auto& viewId : viewIds) {
        const auto extrinsics = scene_->view(viewId)->camera().extrinsics();
        if (problem_->IsParameterBlockConstant(extrinsics) ||
            !problem_->HasParameterBlock(extrinsics)) {
            LOG(ERROR) << "There was a view that could not be found in the "
                          "reconstruction or is set to fixed! "
                          "No covariance estimation possible.";
            return false;
        }

        estViewIds.push_back(viewId);
        covariance_blocks.push_back(std::make_pair(extrinsics, extrinsics));
    }

    ceres::Covariance covariance_estimator{covariance_options_};
    if (!covariance_estimator.Compute(covariance_blocks, problem_.get())) {
        return false;
    }

    for (size_t i{0}; i < estViewIds.size(); ++i) {
        Matrix6d cov;
        covariance_estimator.GetCovarianceMatrixInTangentSpace(
            {covariance_blocks[i].first}, cov.data());
        covariances.insert(std::make_pair(estViewIds[i], cov));
    }

    return true;
}

} // namespace tl
