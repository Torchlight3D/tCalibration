#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tCore/Timer>
#include <tMath/Distribution>
#include <tMath/Eigen/Rotation>
#include <tMvs/Poses/EstimateRotationLagrange>

using namespace tl;

using Eigen::Vector3d;

namespace {
RandomNumberGenerator kRNG{62};
}

class LagrangeDualRotationAveragingTest : public ::testing::Test
{
protected:
    void SetUp() override {}

    void TearDown() override {}

    void TestLagrangeDualRotationEstimator(int num_views, int num_view_pairs,
                                           double relative_rotation_noise,
                                           double mean, double variance,
                                           double rotation_tolerance_degrees)
    {
        // At least a cycle graph
        ASSERT_LE(num_views + 1, num_view_pairs);

        // Set up the cameras
        CreateGTOrientations(num_views);
        CreateRelativeRotations(num_view_pairs, relative_rotation_noise, mean,
                                variance);

        // Initialize estimated orientations
        std::unordered_map<ViewId, Vector3d> estimated_orientations;
        // Or initialized randomly.
        this->InitializeRotationsFromSpanningTree(estimated_orientations);

        // Estimate global orientations
        LagrangeDualRotationEstimator rotation_estimator;

        math::SDPSolver::Options options;
        options.max_iterations = 500;
        options.tolerance = 1e-18;
        // options.solver_type = solver::RBR_BCM;

        rotation_estimator.SetRAOption(options);

        LOG(INFO) << "\nEstimating Global Orientations using Lagrange Dual...";
        Timer timer;
        EXPECT_TRUE(rotation_estimator.EstimateRotations(
            _viewPairs, &estimated_orientations));
        EXPECT_EQ(estimated_orientations.size(), _orientations.size());
        LOG(INFO) << "Elapsed time: " << timer.elapseInSecond();

        rotation_estimator.ComputeErrorBound(_viewPairs);
        const double alpha_max = rotation_estimator.GetErrorBound();
        const double error_bound = math::radToDeg(alpha_max);

        // LOG(INFO) << "Align the rotations and measure the error";
        // // Align the rotations and measure the error
        // geometry::AlignOrientations(orientations_, &estimated_orientations);

        double sum_angular_error = 0.0;
        double min_angular_error = std::numeric_limits<double>::max();
        double max_angular_error = 0.0;

        for (unsigned i = 0; i < _orientations.size(); i++) {
            const auto& rotation = con::FindOrDie(_orientations, i);
            const Vector3d& estimated_rotation =
                con::FindOrDie(estimated_orientations, i);
            const Vector3d relative_rotation = RelativeRotationFromTwoRotations(
                estimated_rotation, rotation, 0.);
            const double angular_error =
                math::radToDeg(relative_rotation.norm());

            sum_angular_error += angular_error;
            min_angular_error = std::min(min_angular_error, angular_error);
            max_angular_error = std::max(max_angular_error, angular_error);
            EXPECT_LT(angular_error, rotation_tolerance_degrees);
            // std::cout << "\n";
            // LOG(INFO) << i << "-th GT        rotation angle = "
            //           << RadToDeg(rotation.norm());
            // LOG(INFO) << i << "-th Estimated rotation angle = "
            //           << RadToDeg(estimated_rotation.norm());
            // LOG(INFO) << i << "-th Angular Residual         = " <<
            // angular_error;
        }

        LOG(INFO) << "\n"
                     "Angular Residual Upperbound: "
                  << error_bound << "Average Angular Residual: "
                  << sum_angular_error / _orientations.size()
                  << "Maximum Angular Residual: " << max_angular_error
                  << "Minimum Angular Residual: " << min_angular_error;
    }

protected:
private:
    void CreateGTOrientations(int num_views)
    {
        // Rotation about the z-axis by 2π/n rad forming a cycle graph
        const Vector3d axis = Vector3d::UnitZ();

        auto angle{0.}; // rad
        for (auto i{0}; i < num_views; ++i) {
            _orientations[i] = angle * axis;
            angle += (2 * pi / num_views);
        }
    }

    void CreateRelativeRotations(int num_view_pairs, double noise, double mean,
                                 double variance)
    {
        RandomNumberGenerator rng;
        NormalDistribution normal_distribution(mean, variance);
        const double relative_rotation_noise = normal_distribution.Eval(noise);
        const double noise_factor = 2.0;

        // Create a set of view id pairs that will contain a spanning tree
        for (size_t i = 1; i < _orientations.size(); i++) {
            const double x = rng.randFloat(std::numeric_limits<double>::min(),
                                           std::numeric_limits<double>::max());
            const double real_noise = normal_distribution.Eval(x);

            const ViewIdPair viewPairId(i - 1, i);
            _viewPairs[viewPairId].visibility_score = rng.randInt(1, 10);
            _viewPairs[viewPairId].rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(_orientations, viewPairId.first),
                con::FindOrDie(_orientations, viewPairId.second),
                noise_factor * real_noise /
                    _viewPairs[viewPairId].visibility_score);
        }

        // Add random edges
        const ViewId maxViewId = static_cast<int>(_orientations.size()) - 1;
        while (_viewPairs.size() < (unsigned)num_view_pairs) {
            ViewIdPair viewPairId{rng.randInt(0, maxViewId),
                                  rng.randInt(0, maxViewId)};

            _viewPairs[viewPairId].visibility_score = rng.randInt(1, 10);
            _viewPairs[viewPairId].rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(_orientations, viewPairId.first),
                con::FindOrDie(_orientations, viewPairId.second),
                noise_factor * relative_rotation_noise /
                    _viewPairs[viewPairId].visibility_score);
        }
    }

    void InitializeRotationsFromSpanningTree(
        std::unordered_map<ViewId, Vector3d>& initial_orientations)
    {
        // initial_orientations[0] = Vector3d::Zero();
        // for (size_t i = 1; i < orientations_.size(); i++) {
        //   initial_orientations[i] = geometry::ApplyRelativeRotation(
        //       FindOrDie(initial_orientations, i - 1),
        //       FindOrDieNoPrint(view_pairs_, ViewIdPair(i - 1,
        //       i)).rotation_2);
        // }
        for (size_t i = 0; i < _orientations.size(); i++) {
            initial_orientations[i] = Vector3d::Zero();
        }
    }

protected:
    std::unordered_map<ViewId, Vector3d> _orientations;
    std::unordered_map<ViewIdPair, ViewPairInfo> _viewPairs;
};

// FIXME: This test case would fail
TEST_F(LagrangeDualRotationAveragingTest, SmallSceneNoNoise)
{
    const int num_views = 4;
    const int num_view_pairs = 6;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.1;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View20WithSmallNoise)
{
    const int num_views = 20;
    const int num_view_pairs = 30;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View20WithLargeNoise)
{
    const int num_views = 20;
    const int num_view_pairs = 30;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View50WithSmallNoise)
{
    const int num_views = 50;
    const int num_view_pairs = 75;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View50WithLargeNoise)
{
    const int num_views = 50;
    const int num_view_pairs = 75;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

// FIXME: Estimated rotation errors are around 2e-6, which is larger than 1e-8
TEST_F(LagrangeDualRotationAveragingTest, View100WithSmallNoise)
{
    const int num_views = 100;
    const int num_view_pairs = 300;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 1e-8;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View100WithLargeNoise)
{
    const int num_views = 100;
    const int num_view_pairs = 300;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View200WithSmallNoise)
{
    const int num_views = 200;
    const int num_view_pairs = 400;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View200WithLargeNoise)
{
    const int num_views = 200;
    const int num_view_pairs = 400;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View500WithSmallNoise)
{
    const int num_views = 500;
    const int num_view_pairs = 2000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View500WithLargeNoise)
{
    const int num_views = 500;
    const int num_view_pairs = 2000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View1000WithSmallNoise)
{
    const int num_views = 1000;
    const int num_view_pairs = 4000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View1000WithLargeNoise)
{
    const int num_views = 1000;
    const int num_view_pairs = 4000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View5000WithSmallNoise)
{
    const int num_views = 5000;
    const int num_view_pairs = 20000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}

TEST_F(LagrangeDualRotationAveragingTest, View5000WithLargeNoise)
{
    const int num_views = 5000;
    const int num_view_pairs = 20000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestLagrangeDualRotationEstimator(num_views, num_view_pairs, noise, mean,
                                      variance, rotation_tolerance_degrees);
}
