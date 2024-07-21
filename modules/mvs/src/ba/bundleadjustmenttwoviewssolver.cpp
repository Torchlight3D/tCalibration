#include "bundleadjustment.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <tCamera/Camera>
#include <tCamera/ReprojectionError>
#include <tMvs/Feature>
#include <tMvs/ViewPairInfo>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

// TODO: Duplicate in BundleAdjustment
void baOptionsToCeresOptions(const BundleAdjustment::Options& ba_options,
                             ceres::Solver::Options& ceres_options)
{
    ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres_options.visibility_clustering_type = ceres::CANONICAL_VIEWS;
    ceres_options.logging_type = ceres::SILENT;
    ceres_options.num_threads = ba_options.num_threads;
    ceres_options.max_num_iterations = ba_options.max_num_iterations;
    // Solver options takes ownership of the ordering so that we can order the
    // BA problem by points and cameras.
    ceres_options.linear_solver_ordering.reset(
        new ceres::ParameterBlockOrdering);
}

// The only intrinsic parameter we want to optimize is the focal length, so we
// keep all intrinsics constant except for focal length by default.
void AddCameraParametersToProblem(bool constant_extrinsic,
                                  bool constant_intrinsic, Camera* camera,
                                  ceres::Problem* problem)
{
    double* extrinsics = camera->rExtrinsics();
    double* intrinsics = camera->rIntrinsics();
    const int num_intrinsics = camera->cameraIntrinsics()->numParameters();

    // Add extrinsics to problem
    problem->AddParameterBlock(extrinsics, Camera::ExtrinsicsSize);
    if (constant_extrinsic) {
        problem->SetParameterBlockConstant(extrinsics);
    }

    // Keep the intrinsics constant if desired.
    if (constant_intrinsic) {
        problem->AddParameterBlock(intrinsics, num_intrinsics);
        problem->SetParameterBlockConstant(intrinsics);
    }
    else {
        // NOTE: Only focal length is variable
        std::vector<int> constant_intrinsics(num_intrinsics - 1);
        std::iota(constant_intrinsics.begin(), constant_intrinsics.end(), 1);

        auto* manifold =
            new ceres::SubsetManifold(num_intrinsics, constant_intrinsics);
        problem->AddParameterBlock(intrinsics, num_intrinsics, manifold);
    }
}

} // namespace

struct AngularEpipolarError
{
    const Eigen::Vector2d point1;
    const Eigen::Vector2d point2;

    AngularEpipolarError(const Eigen::Vector2d& point1,
                         const Eigen::Vector2d& point2)
        : point1(point1), point2(point2)
    {
    }

    template <typename T>
    bool operator()(const T* const rvec, const T* const tvec,
                    T* residuals) const
    {
        using Mat3 = Eigen::Matrix3<T>;
        using Vec3 = Eigen::Vector3<T>;

        const Vec3 p1 = point1.cast<T>().homogeneous();
        const Vec3 p2 = point2.cast<T>().homogeneous();

        Mat3 R;
        ceres::AngleAxisToRotationMatrix(
            rvec, ceres::ColumnMajorAdapter3x3(R.data()));

        // Compute values A and B (Eq. 11 in the paper).
        Eigen::Map<const Vec3> t{tvec};

        // I - t * t'
        const Mat3 translation_term = Mat3::Identity() - t * t.transpose();
        // R' * p2
        const Vec3 Rt_p2 = R.transpose() * p2;
        const T a = p1.dot(translation_term * p1) +
                    (R * p2).dot(translation_term * Rt_p2);
        const T b_sqrt = t.dot(p1.cross(Rt_p2));

        // Ensure the square root is real.
        const T sqrt_term = (a * a) / T(4) - b_sqrt * b_sqrt;
        if (sqrt_term < T(0)) {
            residuals[0] = T(1000);
            return true;
        }

        residuals[0] = a / T(2) - sqrt(sqrt_term);
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector2d& feature1,
                                       const Eigen::Vector2d& feature2)
    {
        constexpr int kResidualSize{1};

        return new ceres::AutoDiffCostFunction<
            AngularEpipolarError, kResidualSize, Vector3d::SizeAtCompileTime,
            Vector3d::SizeAtCompileTime>(
            new AngularEpipolarError(feature1, feature2));
    }
};

struct SampsonError
{
    const Eigen::Vector2d point1;
    const Eigen::Vector2d point2;

    explicit SampsonError(const Eigen::Vector2d& point1,
                          const Eigen::Vector2d& point2)
        : point1(point1), point2(point2)
    {
    }

    template <typename T>
    bool operator()(const T* const fundamental, T* residuals) const
    {
        using Mat3 = Eigen::Matrix3<T>;
        using Vec3 = Eigen::Vector3<T>;
        using Vec4 = Eigen::Vector4<T>;

        const Vec3 p1 = point1.cast<T>().homogeneous();
        const Vec3 p2 = point2.cast<T>().homogeneous();

        const Eigen::Map<const Mat3> F{fundamental};
        const Vec3 epiline_x = F * p1;
        const T numerator_sqrt = p2.dot(epiline_x);
        const Vec4 deno{p2.dot(F.col(0)), p2.dot(F.col(1)), epiline_x[0],
                        epiline_x[1]};

        residuals[0] = numerator_sqrt * numerator_sqrt / deno.squaredNorm();
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector2d& feature1,
                                       const Eigen::Vector2d& feature2)
    {
        constexpr int kResidualSize{1};

        return new ceres::AutoDiffCostFunction<SampsonError, kResidualSize,
                                               Matrix3d::SizeAtCompileTime>(
            new SampsonError(feature1, feature2));
    }
};

// TODO: change to Manifold later
struct FundamentalMatrixParametrization
{
    static constexpr int kDeltaSize{7};
    static constexpr int kFSize{9};

    template <typename T>
    bool operator()(const T* const x, const T* const delta,
                    T* x_plus_delta) const
    {
        const Eigen::Map<const Eigen::Matrix3<T>> F{x};
        const Eigen::Map<const Eigen::Matrix<T, kDeltaSize, 1>> d{delta};

        Eigen::Matrix3<T> rmat1, rmat2;
        ceres::AngleAxisToRotationMatrix(d.segment(0, 3).data(), rmat1.data());
        ceres::AngleAxisToRotationMatrix(d.segment(3, 3).data(), rmat2.data());

        Eigen::JacobiSVD<Eigen::Matrix3<T>> svd{
            F, Eigen::ComputeFullV | Eigen::ComputeFullU};
        const auto& U = svd.matrixU();
        const auto& V = svd.matrixV();
        const auto& singularValues = svd.singularValues();
        // parameter update step (equation 3)
        Eigen::Matrix3<T> U_new = U * rmat1;
        Eigen::Matrix3<T> V_new = V * rmat2;
        const T sigma = singularValues(1) / singularValues(0) + d(6);

        // get back full F
        // fundamental matrix update step (equation 2)
        Eigen::Map<Eigen::Matrix3<T>> F_plus_delta(x_plus_delta);
        F_plus_delta = U_new.col(0) * V_new.col(0).transpose() +
                       sigma * U_new.col(1) * V_new.col(1).transpose();
        return true;
    }
};

// A parameterization for Ceres that keeps a 3-dimensional vector as a unit-norm
// vector throughout optimization.
struct UnitNormThreeVectorManifold
{
    template <typename T>
    bool Plus(const T* const x, const T* const delta, T* x_plus_delta) const
    {
        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = x[2] + delta[2];
        const T sq_norm = x_plus_delta[0] * x_plus_delta[0] +
                          x_plus_delta[1] * x_plus_delta[1] +
                          x_plus_delta[2] * x_plus_delta[2];
        if (sq_norm > T(0.0)) {
            const T norm = ceres::sqrt(sq_norm);
            x_plus_delta[0] /= norm;
            x_plus_delta[1] /= norm;
            x_plus_delta[2] /= norm;
        }

        return true;
    }

    template <typename T>
    bool Minus(const T* y, const T* x, T* y_minus_x) const
    {
        y_minus_x[0] = y[0] - x[0];
        y_minus_x[1] = y[1] - x[1];
        y_minus_x[2] = y[2] - x[2];
        return true;
    }
};

// Calculate symmetric geometric cost terms:
//
// forward_error = D(H * x1, x2)
// backward_error = D(H^-1 * x2, x1)
//
// Templated to be used with autodifferentiation.
template <typename T>
void SymmetricGeometricDistanceTerms(const Eigen::Matrix3<T>& H,
                                     const Eigen::Vector2<T>& x1,
                                     const Eigen::Vector2<T>& x2,
                                     T forward_error[2], T backward_error[2])
{
    using Vec3 = Eigen::Vector3<T>;

    const Vec3 x = x1.homogeneous();
    const Vec3 y = x2.homogeneous();

    Vec3 H_x = H * x;
    Vec3 Hinv_y = H.inverse() * y;

    H_x /= H_x(2);
    Hinv_y /= Hinv_y(2);

    forward_error[0] = H_x(0) - y(0);
    forward_error[1] = H_x(1) - y(1);
    backward_error[0] = Hinv_y(0) - x(0);
    backward_error[1] = Hinv_y(1) - x(1);
}

// Cost functor which computes symmetric geometric distance
// used for homography matrix refinement.
struct HomographySymmetricGeometricCostFunctor
{
    const Eigen::Vector2d x_;
    const Eigen::Vector2d y_;

    HomographySymmetricGeometricCostFunctor(const Eigen::Vector2d& x,
                                            const Eigen::Vector2d& y)
        : x_(std::move(x)), y_(std::move(y))
    {
    }

    template <typename T>
    bool operator()(const T* homography_parameters, T* residuals) const
    {
        using Mat3 = Eigen::Matrix<T, 3, 3>;
        using Vec2 = Eigen::Matrix<T, 2, 1>;

        Mat3 H(homography_parameters);
        Vec2 x(T(x_(0)), T(x_(1)));
        Vec2 y(T(y_(0)), T(y_(1)));

        SymmetricGeometricDistanceTerms<T>(H, x, y, &residuals[0],
                                           &residuals[2]);
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector2d& x,
                                       const Eigen::Vector2d& y)
    {
        return new ceres::AutoDiffCostFunction<
            HomographySymmetricGeometricCostFunctor,
            Vector2d::SizeAtCompileTime + Vector2d::SizeAtCompileTime,
            Matrix3d::SizeAtCompileTime>(
            new HomographySymmetricGeometricCostFunctor(x, y));
    }
};

// Triangulates all 3d points and performs standard bundle adjustment on the
// points and cameras.
BundleAdjustment::Summary BundleAdjustTwoViews(
    const TwoViewBundleAdjustmentOptions& options,
    const std::vector<Feature2D2D>& corrs, Camera* camera1, Camera* camera2,
    std::vector<Eigen::Vector4d>* point3ds)
{
    CHECK_NOTNULL(camera1);
    CHECK_NOTNULL(camera2);
    CHECK_NOTNULL(point3ds);
    CHECK_EQ(point3ds->size(), corrs.size());

    // Set problem options.
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    // Set solver options.
    ceres::Solver::Options solver_options;
    baOptionsToCeresOptions(options.ba_options, solver_options);
    ceres::ParameterBlockOrdering* parameter_ordering =
        solver_options.linear_solver_ordering.get();

    // Add the two cameras as parameter blocks.
    AddCameraParametersToProblem(true, options.constant_camera1_intrinsics,
                                 camera1, &problem);
    AddCameraParametersToProblem(false, options.constant_camera2_intrinsics,
                                 camera2, &problem);

    constexpr int kPointGroup = 0;
    constexpr int kIntrinsicsGroup = 1;
    constexpr int kExtrinsicsGroup = 2;

    parameter_ordering->AddElementToGroup(camera1->rExtrinsics(),
                                          kExtrinsicsGroup);
    parameter_ordering->AddElementToGroup(camera1->rIntrinsics(),
                                          kIntrinsicsGroup);
    parameter_ordering->AddElementToGroup(camera2->rExtrinsics(),
                                          kExtrinsicsGroup);
    parameter_ordering->AddElementToGroup(camera2->rIntrinsics(),
                                          kIntrinsicsGroup);

    // Add triangulated points to the problem.
    for (size_t i = 0; i < point3ds->size(); i++) {
        problem.AddResidualBlock(
            createReprojectionErrorCostFunction(
                camera1->cameraIntrinsicsModel(), corrs[i].feature1.pos),
            nullptr, camera1->rExtrinsics(), camera1->rIntrinsics(),
            point3ds->at(i).data());
        problem.AddResidualBlock(
            createReprojectionErrorCostFunction(
                camera2->cameraIntrinsicsModel(), corrs[i].feature2.pos),
            nullptr, camera2->rExtrinsics(), camera2->rIntrinsics(),
            point3ds->at(i).data());

        parameter_ordering->AddElementToGroup(point3ds->at(i).data(),
                                              kPointGroup);
    }

    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    LOG_IF(INFO, options.ba_options.verbose) << solver_summary.FullReport();

    BundleAdjustment::Summary summary;
    summary.setup_time_in_seconds = 0.;
    summary.solve_time_in_seconds = solver_summary.total_time_in_seconds;
    summary.initial_cost = solver_summary.initial_cost;
    summary.final_cost = solver_summary.final_cost;

    // This only indicates whether the optimization was successfully run and
    // makes no guarantees on the quality or convergence.
    summary.success = solver_summary.termination_type != ceres::FAILURE;

    return summary;
}

BundleAdjustment::Summary BundleAdjustTwoViewsAngular(
    const BundleAdjustment::Options& options,
    const std::vector<Feature2D2D>& corrs, ViewPairInfo* info)
{
    CHECK_NOTNULL(info);

    BundleAdjustment::Summary summary;

    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem{problem_options};

    ceres::Solver::Options solver_options;
    baOptionsToCeresOptions(options, solver_options);
    // Allow Ceres to determine the ordering.
    solver_options.linear_solver_ordering.reset();

    // Add the relative rotation as a parameter block.
    constexpr int kParameterBlockSize{3};
    problem.AddParameterBlock(info->rotation.data(), kParameterBlockSize);
    // Add the position as a parameter block, ensuring that the norm is 1.
    auto* manifold = new ceres::SphereManifold<3>();
    problem.AddParameterBlock(info->position.data(), kParameterBlockSize,
                              manifold);

    // Add all the epipolar constraints from feature matches.
    auto loss = createLossFunction(options.loss_function_type,
                                   options.robust_loss_width);
    for (const auto& corr : corrs) {
        problem.AddResidualBlock(
            AngularEpipolarError::create(corr.feature1.pos, corr.feature2.pos),
            loss.get(), info->rotation.data(), info->position.data());
    }

    summary.setup_time_in_seconds = 0.;

    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    LOG_IF(INFO, options.verbose) << solver_summary.FullReport();

    summary.solve_time_in_seconds = solver_summary.total_time_in_seconds;
    summary.initial_cost = solver_summary.initial_cost;
    summary.final_cost = solver_summary.final_cost;
    summary.success = solver_summary.termination_type != ceres::FAILURE;

    return summary;
}

BundleAdjustment::Summary OptimizeFundamental(
    const BundleAdjustment::Options& options,
    const std::vector<Feature2D2D>& corrs, Eigen::Matrix3d* F)
{
    CHECK_NOTNULL(F);

    ceres::Problem::Options problem_options;
    ceres::Problem problem{problem_options};

    ceres::Solver::Options solver_options;
    baOptionsToCeresOptions(options, solver_options);
    // Allow Ceres to determine the ordering.
    solver_options.linear_solver_ordering.reset();

    auto* fundamental_parametrization =
        new ceres::AutoDiffLocalParameterization<
            FundamentalMatrixParametrization,
            FundamentalMatrixParametrization::kFSize,
            FundamentalMatrixParametrization::kDeltaSize>;
    problem.AddParameterBlock(F->data(),
                              FundamentalMatrixParametrization::kFSize,
                              fundamental_parametrization);

    // Add all the epipolar constraints from feature matches.
    for (const auto& corr : corrs) {
        problem.AddResidualBlock(
            SampsonError::create(corr.feature1.pos, corr.feature2.pos), nullptr,
            F->data());
    }
    // End setup time.

    BundleAdjustment::Summary ba_summary;
    ba_summary.setup_time_in_seconds = 0.;

    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    LOG_IF(INFO, options.verbose) << solver_summary.FullReport();

    ba_summary.solve_time_in_seconds = solver_summary.total_time_in_seconds;
    ba_summary.initial_cost = solver_summary.initial_cost;
    ba_summary.final_cost = solver_summary.final_cost;

    // This only indicates whether the optimization was successfully run and
    // makes no guarantees on the quality or convergence.
    ba_summary.success = solver_summary.termination_type != ceres::FAILURE;
    return ba_summary;
}

BundleAdjustment::Summary OptimizeHomography(
    const BundleAdjustment::Options& options,
    const std::vector<Feature2D2D>& correspondences,
    Eigen::Matrix3d* homography)
{
    CHECK_NOTNULL(homography);

    // Set problem options.
    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

    ceres::Problem problem(problem_options);

    auto loss = createLossFunction(options.loss_function_type,
                                   options.robust_loss_width);
    for (const auto& match : correspondences) {
        auto* cost = new HomographySymmetricGeometricCostFunctor(
            match.feature1.pos, match.feature2.pos);

        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<
                HomographySymmetricGeometricCostFunctor,
                Vector2d::SizeAtCompileTime + Vector2d::SizeAtCompileTime,
                Matrix3d::SizeAtCompileTime>(cost),
            loss.get(), homography->data());
    }

    // Set solver options.
    ceres::Solver::Options solver_options;
    baOptionsToCeresOptions(options, solver_options);
    // Allow Ceres to determine the ordering.
    solver_options.linear_solver_ordering.reset();

    // Solve the problem.
    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    LOG_IF(INFO, options.verbose) << solver_summary.FullReport();

    // Set the BundleAdjustmentSummary.
    BundleAdjustment::Summary summary;
    summary.solve_time_in_seconds = solver_summary.total_time_in_seconds;
    summary.initial_cost = solver_summary.initial_cost;
    summary.final_cost = solver_summary.final_cost;

    // This only indicates whether the optimization was successfully run and
    // makes no guarantees on the quality or convergence.
    summary.success = solver_summary.termination_type != ceres::FAILURE;
    (*homography) /= (*homography)(2, 2);
    return summary;
}

} // namespace tl
