#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/RandomGenerator>
#include <tCore/Timer>
#include <tMath/Distribution>
#include <tMath/Eigen/Rotation>
#include <tMath/SDP/RiemannianStaircaseSDPSolver>
#include <tMvs/Poses/EstimateRotationHybrid>
#include <tMvs/Poses/IRLSRotationRefine>

using namespace tl;

using Eigen::Vector3d;

namespace {
RandomNumberGenerator kRNG{62};
}

class HybridRotationAveragingTest : public ::testing::Test
{
public:
    void TestHybridRotationEstimator(int num_views, int num_view_pairs,
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

        // Estimate global orientations.
        HybridRotationEstimator::Options options;

        // RiemannianStaircaseSDPSolver

        math::SDPSolver::Options sdp_options;
        sdp_options.max_iterations = 100;
        sdp_options.tolerance = 1e-8;

        IRLSRotationLocalRefiner::Options irls_options;
        irls_options.max_num_irls_iterations = 10;

        options.sdp_solver_options = sdp_options;
        options.irls_options = irls_options;

        HybridRotationEstimator rotation_estimator(options);

        Timer timer;
        EXPECT_TRUE(rotation_estimator.EstimateRotations(
            _viewPairs, &estimated_orientations));
        EXPECT_EQ(estimated_orientations.size(), orientations_.size());
        LOG(INFO) << "Elapsed time: " << timer.elapseInSecond();

        // LOG(INFO) << "Align the rotations and measure the error";
        // // Align the rotations and measure the error
        // geometry::AlignOrientations(orientations_, &estimated_orientations);

        double sum_angular_error = 0.0;
        double min_angular_error = std::numeric_limits<double>::max();
        double max_angular_error = 0.0;

        for (unsigned i = 0; i < orientations_.size(); i++) {
            const auto& rotation = con::FindOrDie(orientations_, i);
            const Vector3d& estimated_rotation =
                con::FindOrDie(estimated_orientations, i);
            const Vector3d relative_rotation = RelativeRotationFromTwoRotations(
                estimated_rotation, rotation, 0.);
            const double angular_error =
                math::radToDeg(relative_rotation.norm());

            sum_angular_error += angular_error;
            min_angular_error = std::min(min_angular_error, angular_error);
            max_angular_error = std::max(max_angular_error, angular_error);
            // EXPECT_LT(angular_error, rotation_tolerance_degrees)
            // std::cout << "\n";
            // LOG(INFO) << i << "-th GT        rotation angle = "
            //           << RadToDeg(rotation.norm());
            // LOG(INFO) << i << "-th Estimated rotation angle = "
            //           << RadToDeg(estimated_rotation.norm());
            // LOG(INFO) << i << "-th Angular Residual         = " <<
            // angular_error;
        }

        std::cout << "\n";
        LOG(INFO) << "Average Angular Residual: "
                  << sum_angular_error / orientations_.size();
        LOG(INFO) << "Maximum Angular Residual: " << max_angular_error;
        LOG(INFO) << "Minimum Angular Residual: " << min_angular_error;
    }

protected:
    void SetUp() override {}

    void CreateGTOrientations(int num_views)
    {
        // Rotation about the z-axis by 2π/n rad forming a cycle graph
        const Vector3d axis = Vector3d::UnitZ();

        auto angle{0.}; // rad
        for (auto i{0}; i < num_views; ++i) {
            orientations_[i] = angle * axis;
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
        for (size_t i = 1; i < orientations_.size(); i++) {
            const double x = rng.randFloat(std::numeric_limits<double>::min(),
                                           std::numeric_limits<double>::max());
            const double real_noise = normal_distribution.Eval(x);

            const ViewIdPair view_id_pair(i - 1, i);
            _viewPairs[view_id_pair].visibility_score = rng.randInt(1, 10);
            _viewPairs[view_id_pair].rotation =
                RelativeRotationFromTwoRotations(
                    con::FindOrDie(orientations_, view_id_pair.first),
                    con::FindOrDie(orientations_, view_id_pair.second),
                    noise_factor * real_noise /
                        _viewPairs[view_id_pair].visibility_score);
        }

        // Add random edges
        const ViewId maxViewId = static_cast<int>(orientations_.size()) - 1;
        while (_viewPairs.size() < (unsigned)num_view_pairs) {
            ViewIdPair viewPairId{rng.randInt(0, maxViewId),
                                  rng.randInt(0, maxViewId)};
            _viewPairs[viewPairId].visibility_score = rng.randInt(1, 10);
            _viewPairs[viewPairId].rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(orientations_, viewPairId.first),
                con::FindOrDie(orientations_, viewPairId.second),
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
        for (size_t i = 0; i < orientations_.size(); i++) {
            initial_orientations[i] = Vector3d::Zero();
        }
    }

protected:
    std::unordered_map<ViewId, Vector3d> orientations_;
    std::unordered_map<ViewIdPair, ViewPairInfo> _viewPairs;
};

TEST_F(HybridRotationAveragingTest, smallTestNoNoise)
{
    const int num_views = 4;
    const int num_view_pairs = 6;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.1;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, TwentyViewsTestWithSmallNoise)
{
    const int num_views = 20;
    const int num_view_pairs = 30;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, TwentyViewsTestWithLargeNoise)
{
    const int num_views = 20;
    const int num_view_pairs = 30;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiftyViewsTestWithSmallNoise)
{
    const int num_views = 50;
    const int num_view_pairs = 75;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiftyViewsTestWithLargeNoise)
{
    const int num_views = 50;
    const int num_view_pairs = 75;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, OneHundredViewsTestWithSmallNoise)
{
    const int num_views = 100;
    const int num_view_pairs = 300;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 1e-8;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, OneHundredViewsTestWithLargeNoise)
{
    const int num_views = 100;
    const int num_view_pairs = 300;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, TwoHundredViewsTestWithSmallNoise)
{
    const int num_views = 200;
    const int num_view_pairs = 400;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, TwoHundredViewsTestWithLargeNoise)
{
    const int num_views = 200;
    const int num_view_pairs = 400;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiveHundredViewsTestWithSmallNoise)
{
    const int num_views = 500;
    const int num_view_pairs = 2000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiveHundredViewsTestWithLargeNoise)
{
    const int num_views = 500;
    const int num_view_pairs = 2000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, OneThousandViewsTestWithSmallNoise)
{
    const int num_views = 1000;
    const int num_view_pairs = 4000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, OneThousandViewsTestWithLargeNoise)
{
    const int num_views = 1000;
    const int num_view_pairs = 4000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiveThousandViewsTestWithSmallNoise)
{
    const int num_views = 5000;
    const int num_view_pairs = 20000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.2;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}

TEST_F(HybridRotationAveragingTest, FiveThousandViewsTestWithLargeNoise)
{
    const int num_views = 5000;
    const int num_view_pairs = 20000;
    const double noise = 1.0;
    const double mean = 0.0;
    const double variance = 0.5;
    const double rotation_tolerance_degrees = 4.0;
    TestHybridRotationEstimator(num_views, num_view_pairs, noise, mean,
                                variance, rotation_tolerance_degrees);
}
