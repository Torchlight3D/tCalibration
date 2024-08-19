#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMvs/PnP/UPnP>

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace {
RandomNumberGenerator rng(57);

void AddNoiseToRay(double std_dev, RandomNumberGenerator* rng, Vector3d* proj)
{
    CHECK_NOTNULL(rng);
    const double scale = proj->norm();
    const double noise_x = rng->randNorm(0.0, std_dev);
    const double noise_y = rng->randNorm(0.0, std_dev);

    Quaterniond rot = Quaterniond::FromTwoVectors(Vector3d(0, 0, 1.0), *proj);
    Vector3d noisy_point(noise_x, noise_y, 1);
    noisy_point *= scale;

    *proj = (rot * noisy_point).normalized();
}

} // namespace

struct InputDatum
{
    std::vector<Eigen::Vector3d> ray_origins;
    std::vector<Eigen::Vector3d> ray_directions;
    std::vector<Eigen::Vector3d> world_points;
};

InputDatum ComputeInputDatum(const std::vector<Eigen::Vector3d>& world_points,
                             const std::vector<Eigen::Vector3d>& camera_centers,
                             const Eigen::Quaterniond& expected_rotation,
                             const Eigen::Vector3d& expected_translation)
{
    const int num_points = world_points.size();
    const int num_cameras = camera_centers.size();

    InputDatum input_datum;
    std::vector<Vector3d>& ray_directions = input_datum.ray_directions;
    std::vector<Vector3d>& ray_origins = input_datum.ray_origins;
    ray_directions.reserve(num_points);
    ray_origins.reserve(num_points);

    for (int i = 0; i < num_points; ++i) {
        // Ray origin wrt to coordinate system of the camera rig.
        const Vector3d ray_origin =
            expected_rotation * camera_centers[i % num_cameras] +
            expected_translation;

        ray_origins.emplace_back(std::move(ray_origin));

        // Reproject 3D points into camera frame.
        ray_directions.emplace_back((expected_rotation * world_points[i] +
                                     expected_translation - ray_origins[i])
                                        .normalized());
    }

    input_datum.world_points = world_points;
    return input_datum;
}

bool CheckReprojectionErrors(const InputDatum& input_datum,
                             const Eigen::Quaterniond& soln_rotation,
                             const Eigen::Vector3d& soln_translation,
                             const double kMaxReprojectionError)
{
    const int num_points = input_datum.world_points.size();
    double good_reprojection_errors = true;
    for (int i = 0; i < num_points; ++i) {
        const double reprojection_error = Upnp::ComputeResidual(
            input_datum.ray_origins[i], input_datum.ray_directions[i],
            input_datum.world_points[i], soln_rotation, soln_translation);

        good_reprojection_errors =
            (good_reprojection_errors &&
             (reprojection_error < kMaxReprojectionError));
    }
    return good_reprojection_errors;
}

void TestUpnpPoseEstimationWithNoise(
    const Eigen::Quaterniond& expected_rotation,
    const Eigen::Vector3d& expected_translation,
    double projection_noise_std_dev, double max_reprojection_error,
    double max_rotation_difference, double max_translation_difference,
    InputDatum* input_datum)
{
    const int num_points = input_datum->world_points.size();
    // Add noise to ray.
    if (projection_noise_std_dev > 0.0) {
        for (int i = 0; i < input_datum->world_points.size(); ++i) {
            AddNoiseToRay(projection_noise_std_dev, &rng,
                          &input_datum->ray_directions[i]);
        }
    }

    // Estimate pose.
    std::vector<Quaterniond> solution_rotations;
    std::vector<Vector3d> solution_translations;
    const Upnp::CostParameters upnp_params = Upnp(
        input_datum->ray_origins, input_datum->ray_directions,
        input_datum->world_points, &solution_rotations, &solution_translations);
    const double upnp_cost = Upnp::EvaluateCost(upnp_params, expected_rotation);
    VLOG(3) << "Upnp cost with expected rotation: " << upnp_cost;

    // Check solutions and verify at least one is close to the actual solution.
    const int num_solutions = solution_rotations.size();
    EXPECT_GT(num_solutions, 0);
    bool matched_transform = false;
    for (int i = 0; i < num_solutions; ++i) {
        const bool good_reprojection_errors = CheckReprojectionErrors(
            *input_datum, solution_rotations[i], solution_translations[i],
            max_reprojection_error);
        const double rotation_difference =
            expected_rotation.angularDistance(solution_rotations[i]);
        const bool matched_rotation =
            rotation_difference < max_rotation_difference;
        const double translation_difference =
            (expected_translation - solution_translations[i]).squaredNorm();
        const bool matched_translation =
            translation_difference < max_translation_difference;
        VLOG(3) << "Matched rotation: " << matched_rotation
                << " rotation error [deg]="
                << math::radToDeg(rotation_difference);
        VLOG(3) << "Matched translation: " << matched_translation
                << " translation error=" << translation_difference;
        VLOG(3) << "Good reprojection errors: " << good_reprojection_errors;
        if (matched_rotation && matched_translation &&
            good_reprojection_errors) {
            matched_transform = true;
        }
    }
    EXPECT_TRUE(matched_transform);
}

// Verifies that the cost-function parameters are correct for central cameras.
TEST(UpnpTests, ComputeCostParametersForCentralCameraPoseEstimation)
{
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigin = {Vector3d(2.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    const double kNoise = 0.0;
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigin,
                                               soln_rotation, soln_translation);

    std::vector<Quaterniond> solution_rotations;
    std::vector<Vector3d> solution_translations;
    const Upnp::CostParameters upnp_params = Upnp(
        input_datum.ray_origins, input_datum.ray_directions,
        input_datum.world_points, &solution_rotations, &solution_translations);

    const double upnp_cost = Upnp::EvaluateCost(upnp_params, soln_rotation);
    EXPECT_NEAR(upnp_cost, 0.0, 1e-6);
}

// Verifies that the cost-function parameters are correct for non-central
// cameras.
TEST(UpnpTests, ComputeCostParametersForNonCentralCameraPoseEstimation)
{
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation =
        Quaterniond(AngleAxisd(math::degToRad(13.0), Vector3d(0.0, 0.0, 1.0)));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    const double kNoise = 0.0;
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);

    std::vector<Quaterniond> solution_rotations;
    std::vector<Vector3d> solution_translations;
    const Upnp::CostParameters upnp_params = Upnp(
        input_datum.ray_origins, input_datum.ray_directions,
        input_datum.world_points, &solution_rotations, &solution_translations);

    const double upnp_cost = Upnp::EvaluateCost(upnp_params, soln_rotation);
    EXPECT_NEAR(upnp_cost, 0.0, 1e-6);
}

// Verifies that the computation of the residual is correct.
TEST(UpnpTests, EvaluateResidualAtOptimalSolution)
{
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
}

// Checks the case of a minimal sample and central camera pose estimation.
TEST(UpnpTests, MinimalSampleCentralCameraPoseEstimation)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigin = {Vector3d(2.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigin,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Checks the case of a minimal sample and central camera pose estimation.
TEST(UpnpTests, MinimalSampleCentralCameraPoseEstimationWithNoise)
{
    const double kNoiseStdDev = 0.4 / 512.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1.0);
    const double kMaxAllowedTranslationDifference = 1e-3;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigin = {Vector3d(2.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigin,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Checks the case of a minimal sample and a non-central camera pose estimation.
TEST(UpnpTests, MinimalSampleNonCentralCameraPoseEstimation)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Checks the case of a minimal sample and a non-central camera pose estimation
// with noise.
TEST(UpnpTests, MinimalSampleNonCentralCameraPoseEstimationWithNoise)
{
    const double kNoiseStdDev = 1e-4;
    const double kMaxReprojectionError = 3.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1.0);
    const double kMaxAllowedTranslationDifference = 1e-3;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Checks that the estimator works well using a non-minimal sample of data
// points.
TEST(UpnpTests, NonMinimalSampleCentralCameraPoseEstimation)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {Vector3d(0.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Checks that the estimator works well using a non-minimal sample of data
// points.
TEST(UpnpTests, NonMinimalSampleCentralCameraPoseEstimationWithNoise)
{
    const double kNoiseStdDev = 5e-4;
    const double kMaxReprojectionError = 3.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1.0);
    const double kMaxAllowedTranslationDifference = 1e-3;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {Vector3d(0.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Checks that the estimator works well using a non-minimal sample of data.
TEST(UpnpTests, NonMinimalSampleNonCentralCameraPoseEstimation)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Checks that the estimator works well using a non-minimal sample of data
// points with noise.
TEST(UpnpTests, NonMinimalSampleNonCentralCameraPoseEstimationWithNoise)
{
    const double kNoiseStdDev = 1e-3;
    const double kMaxReprojectionError = 3.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1.0);
    const double kMaxAllowedTranslationDifference = 1e-3;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(13.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Tests estimation of pose when only translation occurs on a central camera.
TEST(UpnpTests, NoRotationOnMinimalSampleAndCentralCamera)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigin = {Vector3d(2.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(0.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigin,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Tests estimation of pose when only translation occurs on a non-central
// camera.
TEST(UpnpTests, NoRotationOnMinimalSampleAndNonCentralCamera)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(0.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Tests estimation of pose when only translation occurs on a central camera.
TEST(UpnpTests, NoRotationOnNonMinimalSampleAndCentralCamera)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigin = {Vector3d(2.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(0.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigin,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Tests estimation of pose when only translation occurs on a central camera.
TEST(UpnpTests, NoRotationOnNonMinimalSampleAndNonCentralCamera)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(0.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(1.0, 1.0, 1.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Tests estimation of pose when only rotation occurs on a central camera.
TEST(UpnpTests, NoTranslationOnMinimalSampleAndCentralCamera)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigin = {Vector3d(2.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(10.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(0.0, 0.0, 0.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigin,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Tests estimation of pose when only rotation occurs on a non-central
// camera.
TEST(UpnpTests, NoTranslationOnMinimalSampleAndNonCentralCamera)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0), Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0), Vector3d(2.0, 1.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(10.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(0.0, 0.0, 0.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Tests estimation of pose when only rotation occurs on a central camera.
TEST(UpnpTests, NoTranslationOnNonMinimalSampleAndCentralCamera)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigin = {Vector3d(2.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(10.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(0.0, 0.0, 0.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigin,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}

// Tests estimation of pose when only translation occurs on a central camera.
TEST(UpnpTests, NoTranslationOnNonMinimalSampleAndNonCentralCamera)
{
    const double kNoiseStdDev = 0.0;
    const double kMaxReprojectionError = 1.0 / 512.0;
    const double kMaxAllowedRotationDifference = math::degToRad(1e-4);
    const double kMaxAllowedTranslationDifference = 1e-6;
    const std::vector<Vector3d> kPoints3d = {
        Vector3d(-1.0, 3.0, 3.0),  Vector3d(1.0, -1.0, 2.0),
        Vector3d(-1.0, 1.0, 2.0),  Vector3d(2.0, 1.0, 3.0),
        Vector3d(-1.0, -3.0, 2.0), Vector3d(1.0, -2.0, 1.0),
        Vector3d(-1.0, 4.0, 2.0),  Vector3d(-2.0, 2.0, 3.0)};
    const std::vector<Vector3d> kImageOrigins = {
        Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0),
        Vector3d(2.0, 0.0, 0.0), Vector3d(3.0, 0.0, 0.0)};
    const Quaterniond soln_rotation = Quaterniond(
        AngleAxisd(math::degToRad(10.0), Vector3d(1.0, 0.0, 1.0).normalized()));
    const Vector3d soln_translation(0.0, 0.0, 0.0);
    // Compute input datum.
    InputDatum input_datum = ComputeInputDatum(kPoints3d, kImageOrigins,
                                               soln_rotation, soln_translation);
    // Execute test.
    TestUpnpPoseEstimationWithNoise(
        soln_rotation, soln_translation, kNoiseStdDev, kMaxReprojectionError,
        kMaxAllowedRotationDifference, kMaxAllowedTranslationDifference,
        &input_datum);
}
