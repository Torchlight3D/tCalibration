#include "bundleadjustment.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <tCamera/CameraIntrinsics>
#include <tCamera/DoubleSphereCameraModel>
#include <tCamera/ExtendedUnifiedCameraModel>
#include <tCamera/OmnidirectionalCameraModel>
#include <tCamera/ReprojectionError>
#include <tCore/ContainerUtils>
#include <tMath/Solvers/LossFunction>
#include <tMVS/Scene>

#include "../residuals/inversereprojectionerror.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::MatrixXd;

namespace {

// TODO: Duplicate in BundleAdjustmentTwoViews
void baOptionsToCeresOptions(const BundleAdjustment::Options& ba_options,
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

        const T sqrt_information = T(1. / sqrt(feature_.depthPriorVar));

        (*residual) =
            sqrt_information * (rotated_point[2] - T(feature_.depthPrior));

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

///------- BundleAdjustment::Impl starts from here
class BundleAdjustment::Impl
{
public:
    Impl();

    void init(const Options& options, Scene* scene);

    void setCameraIntrinsicsParameterization();

    void setCameraExtrinsicsConstant(ViewId id);
    void setCameraPositionConstant(ViewId id);
    void setCameraOrientationConstant(ViewId id);
    void setTzConstant(ViewId id);
    void setCameraExtrinsicsParameterization();

    void setTrackConstant(TrackId id);
    void setTrackVariable(TrackId id);
    void setHomogeneousPointParametrization(TrackId id);

    void setCameraSchurGroups(ViewId id);
    void setTrackSchurGroup(TrackId id);

    void addReprojectionErrorResidual(const Feature& feature, Camera* camera,
                                      Track* track);
    void addInvReprojectionErrorResidual(const Feature& feature,
                                         const Eigen::Vector3d& ref_bearing,
                                         Camera* camera_ref,
                                         Camera* camera_other, Track* track);
    void addPositionPriorErrorResidual(View* view, Camera* camera);
    void addDepthPriorErrorResidual(const Feature& feature, Camera* camera,
                                    Track* track);

    CameraIntrinsics::Ptr intrinsicsOfCamera(CameraId id);

public:
    Options options_;
    Scene* scene_;

    std::unique_ptr<ceres::Problem> problem_;
    ceres::Solver::Options solver_options_;

    std::unique_ptr<ceres::LossFunction> loss_function_;
    std::unique_ptr<ceres::LossFunction> depth_prior_loss_function_;

    ceres::ParameterBlockOrdering* parameter_ordering_;

    std::unordered_set<ViewId> m_optimizedViewIds;
    std::unordered_set<TrackId> m_optimizedTrackIds;
    std::unordered_set<CameraId> m_optimizedCameraIds;

    // Intrinsics groups that have at least 1 camera marked as "const"
    // during optimization. Only the intrinsics that have no optimized
    // cameras are kept as constant during optimization.
    std::unordered_set<CameraId> m_potentialConstantCameraIds;
};

BundleAdjustment::Impl::Impl() {}

void BundleAdjustment::Impl::init(const Options& options, Scene* scene)
{
    options_ = options;

    scene_ = scene;

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

void BundleAdjustment::Impl::setCameraIntrinsicsParameterization()
{
    for (const auto& camId : m_optimizedCameraIds) {
        auto intrinsics = intrinsicsOfCamera(camId);
        auto* parameters = intrinsics->rParameters();

        // Set parameter bound
        // 1. Focal length
        problem_->SetParameterLowerBound(parameters, CameraIntrinsics::Fx, 1.);

        // 2. Distortion
        switch (intrinsics->type()) {
            case CameraIntrinsicsType::DoubleSphere: {
                problem_->SetParameterLowerBound(
                    parameters, DoubleSphereCameraModel::Xi, -1.);
                problem_->SetParameterUpperBound(
                    parameters, DoubleSphereCameraModel::Xi, 1.);
                problem_->SetParameterLowerBound(
                    parameters, DoubleSphereCameraModel::Alpha, 0.);
                problem_->SetParameterUpperBound(
                    parameters, DoubleSphereCameraModel::Alpha, 1.);
            } break;
            case CameraIntrinsicsType::ExtendedUnified: {
                problem_->SetParameterLowerBound(
                    parameters, ExtendedUnifiedCameraModel::Alpha, 0.);
                problem_->SetParameterUpperBound(
                    parameters, ExtendedUnifiedCameraModel::Alpha, 1.);
                problem_->SetParameterLowerBound(
                    parameters, ExtendedUnifiedCameraModel::Beta, 0.1);
            } break;
            case CameraIntrinsicsType::Omnidirectional: {
                // Do nothing now
            } break;
            default:
                break;
        }

        // Set the constant parameters if any are requested.
        const std::vector<int> const_indices =
            intrinsics->fixedParameterIndices(options_.intrinsics_to_optimize);

        if (const_indices.size() == intrinsics->numParameters()) {
            problem_->SetParameterBlockConstant(parameters);
        }
        else if (!const_indices.empty()) {
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

        const auto camera_intrinsics = intrinsicsOfCamera(camId);
        problem_->SetParameterBlockConstant(camera_intrinsics->parameters());
    }
}

void BundleAdjustment::Impl::setCameraExtrinsicsConstant(ViewId id)
{
    const auto& camera = scene_->view(id)->camera();
    problem_->SetParameterBlockConstant(camera.extrinsics());
}

void BundleAdjustment::Impl::setCameraPositionConstant(ViewId id)
{
    const std::vector parameters{Camera::Position + 0, Camera::Position + 1,
                                 Camera::Position + 2};
    auto* manifold =
        new ceres::SubsetManifold(Camera::ExtrinsicsSize, parameters);
    auto& camera = scene_->rView(id)->rCamera();
    problem_->SetManifold(camera.rExtrinsics(), manifold);
}

void BundleAdjustment::Impl::setCameraOrientationConstant(ViewId id)
{
    const std::vector parameters{Camera::Orientation + 0,
                                 Camera::Orientation + 1,
                                 Camera::Orientation + 2};
    auto* manifold =
        new ceres::SubsetManifold(Camera::ExtrinsicsSize, parameters);
    auto& camera = scene_->rView(id)->rCamera();
    problem_->SetManifold(camera.rExtrinsics(), manifold);
}

void BundleAdjustment::Impl::setTzConstant(ViewId id)
{
    const std::vector parameters{Camera::Position + 2};
    auto* manifold =
        new ceres::SubsetManifold(Camera::ExtrinsicsSize, parameters);
    auto& camera = scene_->rView(id)->rCamera();
    problem_->SetManifold(camera.rExtrinsics(), manifold);
}

void BundleAdjustment::Impl::setCameraExtrinsicsParameterization()
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

void BundleAdjustment::Impl::setTrackConstant(TrackId trackId)
{
    const auto* track = scene_->track(trackId);
    if (options_.use_inverse_depth_parametrization) {
        problem_->SetParameterBlockConstant(track->inverseDepth());
    }
    else {
        problem_->SetParameterBlockConstant(track->position().data());
    }
}

void BundleAdjustment::Impl::setTrackVariable(TrackId trackId)
{
    auto* track = scene_->rTrack(trackId);
    if (options_.use_inverse_depth_parametrization) {
        problem_->SetParameterBlockVariable(track->rInverseDepth());
    }
    else {
        problem_->SetParameterBlockVariable(track->rPosition().data());
    }
}

void BundleAdjustment::Impl::setHomogeneousPointParametrization(TrackId trackId)
{
    auto* track = scene_->rTrack(trackId);
    auto* pointManifold = new ceres::SphereManifold<4>();
    problem_->SetManifold(track->rPosition().data(), pointManifold);
}

void BundleAdjustment::Impl::setCameraSchurGroups(ViewId viewId)
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

void BundleAdjustment::Impl::setTrackSchurGroup(TrackId trackId)
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

void BundleAdjustment::Impl::addReprojectionErrorResidual(
    const Feature& feature, Camera* camera, Track* track)
{
    // Add the residual for the track to the problem. The shared intrinsics
    // parameter block will be set to constant after the loop if no
    // optimized cameras share the same camera intrinsics.
    problem_->AddResidualBlock(
        createReprojectionErrorCostFunction(camera->cameraIntrinsicsModel(),
                                            feature.pos),
        loss_function_.get(), camera->rExtrinsics(), camera->rIntrinsics(),
        track->rPosition().data());
}

void BundleAdjustment::Impl::addInvReprojectionErrorResidual(
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

void BundleAdjustment::Impl::addPositionPriorErrorResidual(View* view,
                                                           Camera* camera)
{
    // Adds a position priors to the camera poses. This can for example be a
    // GPS position.
    problem_->AddResidualBlock(
        PositionError::create(view->positionPrior(), view->positionPriorSqrt()),
        nullptr, camera->rExtrinsics());
}

void BundleAdjustment::Impl::addDepthPriorErrorResidual(const Feature& feature,
                                                        Camera* camera,
                                                        Track* track)
{
    problem_->AddResidualBlock(
        DepthPriorError::create(feature), depth_prior_loss_function_.get(),
        camera->rExtrinsics(), track->rPosition().data());
}

CameraIntrinsics::Ptr BundleAdjustment::Impl::intrinsicsOfCamera(CameraId camId)
{
    const auto sharedCameraViewIds = scene_->sharedCameraViewIds(camId);
    CHECK(!sharedCameraViewIds.empty());

    // Use first viewId as representative
    const auto viewId = *sharedCameraViewIds.begin();
    return scene_->rView(viewId)->rCamera().cameraIntrinsics();
}

///------- BundleAdjustment starts from here
BundleAdjustment::BundleAdjustment(const Options& options, Scene* scene)
    : d(std::make_unique<Impl>())
{
    CHECK_NOTNULL(scene);
    d->init(options, scene);
}

BundleAdjustment::~BundleAdjustment() = default;

void BundleAdjustment::addView(ViewId viewId)
{
    auto* view = CHECK_NOTNULL(d->scene_->rView(viewId));
    if (!view->estimated() || d->m_optimizedViewIds.contains(viewId)) {
        return;
    }

    d->m_optimizedViewIds.emplace(viewId);

    d->setCameraSchurGroups(viewId);

    d->m_optimizedCameraIds.emplace(d->scene_->cameraId(viewId));

    // Add residuals for all tracks in the view.
    auto& camera = view->rCamera();
    for (const auto& trackId : view->trackIds()) {
        const auto* feature = CHECK_NOTNULL(view->featureOf(trackId));
        auto* track = CHECK_NOTNULL(d->scene_->rTrack(trackId));
        // Only consider tracks with an estimated 3d point.
        if (!track->estimated()) {
            continue;
        }

        // Add the reprojection error to the optimization.
        d->addReprojectionErrorResidual(*feature, &camera, track);

        // Add the point to group 0.
        d->setTrackConstant(trackId);

        // Add depth priors if available
        if (d->options_.use_depth_priors && feature->depthPrior != 0.) {
            d->addDepthPriorErrorResidual(*feature, &camera, track);
        }
    }

    // Add a position prior if available
    if (d->options_.use_position_priors && view->hasPositionPrior()) {
        d->addPositionPriorErrorResidual(view, &camera);
    }
}

void BundleAdjustment::addTrack(TrackId trackId)
{
    auto* track = CHECK_NOTNULL(d->scene_->rTrack(trackId));
    if (!track->estimated() || d->m_optimizedTrackIds.contains(trackId)) {
        return;
    }

    d->m_optimizedTrackIds.emplace(trackId);

    // Add all observations of the track to the problem.
    const auto& observedViewIds = track->viewIds();
    for (const auto& viewId : observedViewIds) {
        auto* view = CHECK_NOTNULL(d->scene_->rView(viewId));
        if (d->m_optimizedViewIds.contains(viewId) || !view->estimated()) {
            continue;
        }

        const auto* feature = CHECK_NOTNULL(view->featureOf(trackId));
        auto& camera = view->rCamera();

        d->addReprojectionErrorResidual(*feature, &camera, track);

        // Any camera that reaches this point was not added by addView() and
        // so we want to mark it as constant.
        d->setCameraExtrinsicsConstant(viewId);

        // Mark the camera intrinsics as "potentially constant." We only set
        // the parameter block to constant if the shared intrinsics are not
        // shared with cameras that are being optimized.
        const auto camId = d->scene_->cameraId(viewId);
        d->m_potentialConstantCameraIds.emplace(camId);
    }

    d->setTrackVariable(trackId);
    d->setTrackSchurGroup(trackId);

    if (d->options_.use_homogeneous_local_point_parametrization) {
        d->setHomogeneousPointParametrization(trackId);
    }
}

void BundleAdjustment::setFixedView(ViewId id)
{
    d->setCameraExtrinsicsConstant(id);
}

BundleAdjustment::Summary BundleAdjustment::optimize()
{
    // Set extrinsics parameterization of the camera poses. This will set
    // orientation and/or positions as constant if desired.
    d->setCameraExtrinsicsParameterization();

    // Set intrinsics group parameterization. This will control which of the
    // intrinsics parameters or optimized or held constant. Note that each
    // camera intrinsics group may be a different camera and/or a different
    // camera intrinsics model.
    d->setCameraIntrinsicsParameterization();

    // NOTE: Use the reverse BA order (i.e., using cameras then points) is a
    // good idea for inner iterations.
    if (d->solver_options_.use_inner_iterations &&
        !d->options_.use_inverse_depth_parametrization) {
        d->solver_options_.inner_iteration_ordering.reset(
            new ceres::ParameterBlockOrdering(*d->parameter_ordering_));
        d->solver_options_.inner_iteration_ordering->Reverse();
    }

    ceres::Solver::Summary ceres_summary;
    ceres::Solve(d->solver_options_, d->problem_.get(), &ceres_summary);
    LOG_IF(INFO, d->options_.verbose) << ceres_summary.FullReport();

    Summary ba_summary;
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

bool BundleAdjustment::calcCovarianceForTrack(TrackId trackId,
                                              Eigen::Matrix3d* covariance) const
{
    // Default
    *covariance = Matrix3d::Identity();

    const auto* track = d->scene_->track(trackId);
    const auto* position = track->position().data();
    if (d->problem_->IsParameterBlockConstant(position) ||
        !d->problem_->HasParameterBlock(position)) {
        return false;
    }

    const std::vector covariance_blocks{std::make_pair(position, position)};

    ceres::Covariance::Options opts;
    ceres::Covariance estimator{opts};
    if (!estimator.Compute(covariance_blocks, d->problem_.get())) {
        return false;
    }

    const std::vector parameter_blocks{position};
    return estimator.GetCovarianceMatrixInTangentSpace(parameter_blocks,
                                                       (*covariance).data());
}

bool BundleAdjustment::calcCovarianceForTracks(
    const std::vector<TrackId>& trackIds,
    std::map<TrackId, Eigen::Matrix3d>* covariances) const
{
    // Default
    covariances->clear();

    std::vector<TrackId> estTrackIds;
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    for (const auto& trackId : trackIds) {
        const auto* track = d->scene_->track(trackId);
        const auto* position = track->position().data();
        if (d->problem_->IsParameterBlockConstant(position) ||
            !d->problem_->HasParameterBlock(position)) {
            LOG(ERROR) << "There was a track that could not be found in the "
                          "scene or is set to fixed! "
                          "No covariance estimation possible.";
            return false;
        }

        estTrackIds.emplace_back(trackId);
        covariance_blocks.emplace_back(position, position);
    }

    ceres::Covariance::Options opts;
    ceres::Covariance estimator{opts};
    if (!estimator.Compute(covariance_blocks, d->problem_.get())) {
        return false;
    }

    for (size_t i{0}; i < estTrackIds.size(); ++i) {
        const std::vector parameter_blocks{covariance_blocks[i].first};
        Matrix3d cov;
        if (!estimator.GetCovarianceMatrixInTangentSpace(parameter_blocks,
                                                         cov.data())) {
            return false;
        }
        covariances->insert({estTrackIds[i], cov});
    }

    return true;
}

bool BundleAdjustment::calcCovarianceForView(ViewId viewId,
                                             Matrix6d* covariance) const
{
    // Default
    *covariance = Matrix6d::Identity();

    const auto* extrinsics = d->scene_->view(viewId)->camera().extrinsics();
    if (d->problem_->IsParameterBlockConstant(extrinsics) ||
        !d->problem_->HasParameterBlock(extrinsics)) {
        return false;
    }

    const std::vector covariance_blocks{std::make_pair(extrinsics, extrinsics)};

    ceres::Covariance::Options opts;
    ceres::Covariance estimator{opts};
    if (!estimator.Compute(covariance_blocks, d->problem_.get())) {
        return false;
    }

    const std::vector parameter_blocks{extrinsics};
    return estimator.GetCovarianceMatrixInTangentSpace(parameter_blocks,
                                                       (*covariance).data());
}

bool BundleAdjustment::calcCovarianceForViews(
    const std::vector<ViewId>& viewIds,
    std::map<ViewId, Matrix6d>* covariances) const
{
    // Default
    covariances->clear();

    std::vector<ViewId> estViewIds;
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    for (const auto& viewId : viewIds) {
        const auto* extrinsics = d->scene_->view(viewId)->camera().extrinsics();
        if (d->problem_->IsParameterBlockConstant(extrinsics) ||
            !d->problem_->HasParameterBlock(extrinsics)) {
            LOG(ERROR) << "There was a view that could not be found in the "
                          "scene or is set to fixed! "
                          "No covariance estimation possible.";
            return false;
        }

        estViewIds.emplace_back(viewId);
        covariance_blocks.emplace_back(extrinsics, extrinsics);
    }

    ceres::Covariance::Options opts;
    ceres::Covariance estimator{opts};
    if (!estimator.Compute(covariance_blocks, d->problem_.get())) {
        return false;
    }

    for (size_t i{0}; i < estViewIds.size(); ++i) {
        const std::vector parameter_blocks{covariance_blocks[i].first};
        Matrix6d cov;
        if (!estimator.GetCovarianceMatrixInTangentSpace(parameter_blocks,
                                                         cov.data())) {
            return false;
        }
        covariances->insert({estViewIds[i], cov});
    }

    return true;
}

bool BundleAdjustment::calcCovarianceForCamera(
    CameraId id, Eigen::MatrixXd* covariance) const
{
    // No default

    const auto* camera = d->scene_->camera(id);
    if (!camera) {
        return false;
    }

    const auto* intrinsics = camera->intrinsics();
    if (d->problem_->IsParameterBlockConstant(intrinsics) ||
        !d->problem_->HasParameterBlock(intrinsics)) {
        return false;
    }

    const std::vector covariance_blocks{std::make_pair(intrinsics, intrinsics)};

    ceres::Covariance::Options opts;
    ceres::Covariance estimator{opts};
    if (!estimator.Compute(covariance_blocks, d->problem_.get())) {
        return false;
    }

    const auto paramCount = camera->cameraIntrinsics()->numParameters();
    *covariance = MatrixXd::Zero(paramCount, paramCount);
    const std::vector parameter_blocks{intrinsics};

    return estimator.GetCovarianceMatrixInTangentSpace(parameter_blocks,
                                                       (*covariance).data());
}

} // namespace tl
