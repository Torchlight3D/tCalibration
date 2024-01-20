#include "bundle_adjustment_two_views_solver.h"

#include <ceres/ceres.h>

#include <tCamera/Camera>
#include <tCamera/ReprojectionError>
#include <tMvs/FeatureCorrespondence>
#include <tMvs/StereoViewInfo>

namespace tl {

namespace {

void baOptionsToCeresOptions(const BundleAdjustmentOptions& ba_options,
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
    const Eigen::Vector2d feature1_;
    const Eigen::Vector2d feature2_;

    AngularEpipolarError(const Eigen::Vector2d& feature1,
                         const Eigen::Vector2d& feature2)
        : feature1_(feature1), feature2_(feature2)
    {
    }

    template <typename T>
    bool operator()(const T* const rvec, const T* const translation,
                    T* residual) const
    {
        // double -> T
        const Eigen::Vector3<T> feature1{T(feature1_(0)), T(feature1_(1)),
                                         T(1)};
        const Eigen::Vector3<T> feature2{T(feature2_(0)), T(feature2_(1)),
                                         T(1)};

        // angle-axis -> rotation matrix
        Eigen::Matrix3<T> rmat;
        ceres::AngleAxisToRotationMatrix(
            rvec, ceres::ColumnMajorAdapter3x3(rmat.data()));

        // Compute values A and B (Eq. 11 in the paper).
        Eigen::Map<const Eigen::Vector3<T>> translation_map(translation);
        const Eigen::Matrix3<T> translation_term =
            Eigen::Matrix3<T>::Identity() -
            translation_map * translation_map.transpose();
        const T a = feature1.dot(translation_term * feature1) +
                    (rmat * feature2)
                        .dot(translation_term * (rmat.transpose() * feature2));
        const T b_sqrt =
            translation_map.dot(feature1.cross(rmat.transpose() * feature2));

        // Ensure the square root is real.
        const T sqrt_term = a * a / T(4) - b_sqrt * b_sqrt;
        if (sqrt_term < T(0)) {
            return false;
        }

        residual[0] = a / T(2) - sqrt(sqrt_term);
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector2d& feature1,
                                       const Eigen::Vector2d& feature2)
    {
        constexpr int kResidualSize{1};
        constexpr int kRotationSize{3};
        constexpr int kTranslationSize{3};
        return new ceres::AutoDiffCostFunction<AngularEpipolarError,
                                               kResidualSize, kRotationSize,
                                               kTranslationSize>(
            new AngularEpipolarError(feature1, feature2));
    }
};

struct SampsonError
{
    const Eigen::Vector2d feature1_;
    const Eigen::Vector2d feature2_;

    explicit SampsonError(const Eigen::Vector2d& feature1,
                          const Eigen::Vector2d& feature2)
        : feature1_(feature1), feature2_(feature2)
    {
    }

    template <typename T>
    bool operator()(const T* const fundamental, T* residual) const
    {
        const Eigen::Map<const Eigen::Matrix3<T>> F(fundamental);
        const Eigen::Vector3<T> epiline_x = F * feature1_.homogeneous();
        const T numerator_sqrt = feature2_.homogeneous().dot(epiline_x);
        const Eigen::Vector4<T> denominator{
            feature2_.homogeneous().dot(F.col(0)),
            feature2_.homogeneous().dot(F.col(1)), epiline_x[0], epiline_x[1]};

        residual[0] =
            numerator_sqrt * numerator_sqrt / denominator.squaredNorm();
        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector2d& feature1,
                                       const Eigen::Vector2d& feature2)
    {
        constexpr int kResidualSize{1};
        constexpr int kFundamentalMatrixSize{9};
        return new ceres::AutoDiffCostFunction<SampsonError, kResidualSize,
                                               kFundamentalMatrixSize>(
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

// Triangulates all 3d points and performs standard bundle adjustment on the
// points and cameras.
BundleAdjustmentSummary BundleAdjustTwoViews(
    const TwoViewBundleAdjustmentOptions& options,
    const std::vector<FeatureCorrespondence>& corrs, Camera* camera1,
    Camera* camera2, std::vector<Eigen::Vector4d>* point3ds)
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
                camera1->cameraIntrinsicsModel(), corrs[i].feature1.point_),
            nullptr, camera1->rExtrinsics(), camera1->rIntrinsics(),
            point3ds->at(i).data());
        problem.AddResidualBlock(
            createReprojectionErrorCostFunction(
                camera2->cameraIntrinsicsModel(), corrs[i].feature2.point_),
            nullptr, camera2->rExtrinsics(), camera2->rIntrinsics(),
            point3ds->at(i).data());

        parameter_ordering->AddElementToGroup(point3ds->at(i).data(),
                                              kPointGroup);
    }

    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    LOG_IF(INFO, options.ba_options.verbose) << solver_summary.FullReport();

    BundleAdjustmentSummary summary;
    summary.setup_time_in_seconds = 0.;
    summary.solve_time_in_seconds = solver_summary.total_time_in_seconds;
    summary.initial_cost = solver_summary.initial_cost;
    summary.final_cost = solver_summary.final_cost;

    // This only indicates whether the optimization was successfully run and
    // makes no guarantees on the quality or convergence.
    summary.success = solver_summary.termination_type != ceres::FAILURE;

    return summary;
}

BundleAdjustmentSummary BundleAdjustTwoViewsAngular(
    const BundleAdjustmentOptions& options,
    const std::vector<FeatureCorrespondence>& corrs, TwoViewInfo* info)
{
    CHECK_NOTNULL(info);

    BundleAdjustmentSummary summary;

    // Set problem options.
    ceres::Problem::Options problem_options;
    ceres::Problem problem{problem_options};

    // Set solver options.
    ceres::Solver::Options solver_options;
    baOptionsToCeresOptions(options, solver_options);
    // Allow Ceres to determine the ordering.
    solver_options.linear_solver_ordering.reset();

    // Add the relative rotation as a parameter block.
    constexpr int kParameterBlockSize{3};
    problem.AddParameterBlock(info->rotation.data(), kParameterBlockSize);
    // Add the position as a parameter block, ensuring that the norm is 1.
    auto* manifold =
        new ceres::AutoDiffManifold<UnitNormThreeVectorManifold, 3, 3>;
    problem.AddParameterBlock(info->position.data(), kParameterBlockSize,
                              manifold);

    // Add all the epipolar constraints from feature matches.
    for (const auto& corr : corrs) {
        problem.AddResidualBlock(
            AngularEpipolarError::create(corr.feature1.point_,
                                         corr.feature2.point_),
            nullptr, info->rotation.data(), info->position.data());
    }
    // End setup time.
    summary.setup_time_in_seconds = 0.;

    // Solve the problem.
    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    LOG_IF(INFO, options.verbose) << solver_summary.FullReport();

    // Set the BundleAdjustmentSummary.
    summary.solve_time_in_seconds = solver_summary.total_time_in_seconds;
    summary.initial_cost = solver_summary.initial_cost;
    summary.final_cost = solver_summary.final_cost;

    // This only indicates whether the optimization was successfully run and
    // makes no guarantees on the quality or convergence.
    summary.success = solver_summary.termination_type != ceres::FAILURE;
    return summary;
}

BundleAdjustmentSummary OptimizeFundamentalMatrix(
    const BundleAdjustmentOptions& options,
    const std::vector<FeatureCorrespondence>& corrs, Eigen::Matrix3d* F)
{
    CHECK_NOTNULL(F);

    // Set problem options.
    ceres::Problem::Options problem_options;
    ceres::Problem problem{problem_options};

    // Set solver options.
    ceres::Solver::Options solver_options;
    baOptionsToCeresOptions(options, solver_options);
    // Allow Ceres to determine the ordering.
    solver_options.linear_solver_ordering.reset();

    // Add the fundamental matrix as a parameter block.

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
            SampsonError::create(corr.feature1.point_, corr.feature2.point_),
            nullptr, F->data());
    }
    // End setup time.

    BundleAdjustmentSummary ba_summary;
    ba_summary.setup_time_in_seconds = 0.;

    // Solve the problem.
    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    LOG_IF(INFO, options.verbose) << solver_summary.FullReport();

    // Set the BundleAdjustmentSummary.
    ba_summary.solve_time_in_seconds = solver_summary.total_time_in_seconds;
    ba_summary.initial_cost = solver_summary.initial_cost;
    ba_summary.final_cost = solver_summary.final_cost;

    // This only indicates whether the optimization was successfully run and
    // makes no guarantees on the quality or convergence.
    ba_summary.success = solver_summary.termination_type != ceres::FAILURE;
    return ba_summary;
}

} // namespace tl
