#include <Eigen/Geometry>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMvs/Registration/AlignPointCloud>

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {
constexpr double kEpsilon = 1e-6;

RandomNumberGenerator kRNG{50};
} // namespace

void UmeyamaSimpleTest()
{
    const std::vector points1{
        Vector3d{0.4, -3.105, 2.147}, {1.293, 7.1982, -.068},
        {-5.34, 0.708, -3.69},        {-.345, 1.987, 0.936},
        {0.93, 1.45, 1.079},          {-3.15, -4.73, 2.49},
        {2.401, -2.03, -1.87},        {3.192, -.573, 0.1},
        {-2.53, 3.07, -5.19}};

    const Matrix3d R =
        AngleAxisd{math::degToRad(15.), Vector3d{1., -2.7, 1.9}.normalized()}
            .toRotationMatrix();
    const Vector3d t{0., 2., 2.};
    constexpr double kScale = 1.5;

    // Transform the points2.
    std::vector<Vector3d> points2;
    for (const auto& point : points1) {
        points2.emplace_back(kScale * R * point + t);
    }

    // Compute the similarity transformation.
    Matrix3d R_est;
    Vector3d t_est;
    double scale_est;
    AlignPointCloudsUmeyama(points1, points2, &R_est, &t_est, &scale_est);

    // Ensure the calculated transformation is the same as the one we set.
    ASSERT_TRUE(R_est.isApprox(R, kEpsilon));
    ASSERT_TRUE(t_est.isApprox(t, kEpsilon));
    ASSERT_LT(std::abs(kScale - scale_est), kEpsilon);
}

// It is not easy to find a formula for the weights that always works
// We need to make some statistics to see if the function works in most cases
//
// This test adds some noise on a given fraction of the points
// We try to estimate the weights from the errors of the first pass (setting
// weights to 1)
// We can expect that the errors are where the noise was added
// From this first result we can set a weight for each point.
// The weight is given by 1.0/(1.0 + distance)
// Where distance is the distance between points2[i] and s * R * points1[i] + T
// estimated by a first pass
// To see if the new estimation from these weights is better
// we need to check that the new parameters are closer to the expected
// parameters
void UmeyamaWithWeigthsAndNoise()
{
    // Make some statistics
    // 1000 is not a problem the algorith is fast
    constexpr size_t kNumIterations = 1000;

    // 15 % of noise
    constexpr auto kNoiseRatio = 0.15f;

    // Percentage to consider the test succeeds (is it enough ?)
    constexpr float kTestsSucceeded = 95.0f;

    size_t succeeded = 0;
    for (size_t iteration = 0; iteration < kNumIterations; ++iteration) {
        // At least 4 points are required
        const auto numPoints = kRNG.randInt(size_t{4}, size_t{1000});

        std::vector<Vector3d> points1(numPoints);
        for (size_t i{0}; i < numPoints; ++i) {
            points1[i] = Vector3d::Random();
        }

        std::vector<double> weights(numPoints, 1.);

        const Matrix3d R = AngleAxisd{math::degToRad(kRNG.randFloat(0., 360.)),
                                      Vector3d::Random().normalized()}
                               .toRotationMatrix();
        const Vector3d t = Vector3d::Random();
        const double scale = kRNG.randFloat(1e-3, 10.);

        // Transform the points.
        std::vector<Vector3d> points2;
        for (const auto& point : points1) {
            points2.emplace_back(scale * R * point + t);
        }

        // Add noise on scale, point and translation
        for (size_t i = 0, end = static_cast<size_t>(kNoiseRatio * numPoints);
             i < end; ++i) {
            const size_t k = kRNG.randInt(size_t{0}, numPoints - 1);
            const double noiseOnScale = scale + kRNG.randFloat(0., 10.);
            points2[k] = noiseOnScale * R * (points1[k] + Vector3d::Random()) +
                         t + Vector3d::Random();
        }

        // We need to find some weights
        Matrix3d rotation_noisy;
        Vector3d translation_noisy;
        double scale_noisy;
        AlignPointCloudsUmeyamaWithWeights(points1, points2, weights,
                                           &rotation_noisy, &translation_noisy,
                                           &scale_noisy);

        for (size_t i = 0; i < numPoints; ++i) {
            const double dist =
                (points2[i] - (scale_noisy * rotation_noisy * points1[i] +
                               translation_noisy))
                    .norm();
            weights[i] = 1. / (1. + dist);
        }

        Matrix3d rotation_weighted;
        Vector3d translation_weighted;
        double scale_weighted;
        AlignPointCloudsUmeyamaWithWeights(
            points1, points2, weights, &rotation_weighted,
            &translation_weighted, &scale_weighted);

        // Check if the parameters are closer to real parameters ?
        const bool condition_on_scale =
            (std::abs(scale_weighted - scale) < std::abs(scale_noisy - scale));
        const bool condition_on_translation =
            (t - translation_weighted).norm() < (t - translation_noisy).norm();
        const bool condition_on_rotation =
            (R - rotation_weighted).norm() < (R - rotation_noisy).norm();

        if (condition_on_scale && condition_on_translation &&
            condition_on_rotation)
            ++succeeded;
    }

    ASSERT_LE(kTestsSucceeded, 100. * succeeded / (0. + kNumIterations));
}

TEST(AlignPointCloudsUmeyama, SimpleTest) { UmeyamaSimpleTest(); }

TEST(AlignPointCloudsUmeyamaWithWeights, WeightsAndNoise)
{
    UmeyamaWithWeigthsAndNoise();
}
