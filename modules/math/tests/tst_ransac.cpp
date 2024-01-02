#include <gtest/gtest.h>

#include <AxMath/RandomGenerator>
#include <AxMath/Ransac>
#include "test_utils.h"

using namespace thoht;

namespace {
RandomNumberGenerator kRNG(46);
}

// TODO: use Test Fixture
// class RansacTest : public ::testing::Test
//{
// public:
//     static std::vector<Point>* input_points;

// protected:
//     void SetUp() override {

//    }

//    void TearDown() override {

//    }
//};

TEST(RansacTest, LineFitting)
{
    // Create a set of points along y=x with a small random pertubation.
    std::vector<Point> input_points;
    for (int i = 0; i < 10000; ++i) {
        if (i % 2 == 0) {
            double noise_x = kRNG.RandGaussian(0.0, 0.1);
            double noise_y = kRNG.RandGaussian(0.0, 0.1);
            input_points.push_back(Point(i + noise_x, i + noise_y));
        }
        else {
            double noise_x = kRNG.RandDouble(0.0, 10000);
            double noise_y = kRNG.RandDouble(0.0, 10000);
            input_points.push_back(Point(noise_x, noise_y));
        }
    }

    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    params.error_thresh = 0.5;

    LineEstimator line_estimator;
    Ransac<LineEstimator> ransac_line(params, line_estimator);
    ransac_line.Initialize();

    Line line;
    SacSummary summary;
    CHECK(ransac_line.Estimate(input_points, &line, &summary));
    ASSERT_LT(fabs(line.m - 1.0), 0.1);
}

TEST(RansacTest, TerminationNumInliers)
{
    // Create a set of points along y=x with a small random pertubation.
    std::vector<Point> input_points;
    for (int i = 0; i < 10000; ++i) {
        if (i % 2 == 0) {
            double noise_x = kRNG.RandGaussian(0.0, 0.1);
            double noise_y = kRNG.RandGaussian(0.0, 0.1);
            input_points.push_back(Point(i + noise_x, i + noise_y));
        }
        else {
            double noise_x = kRNG.RandDouble(0.0, 10000);
            double noise_y = kRNG.RandDouble(0.0, 10000);
            input_points.push_back(Point(noise_x, noise_y));
        }
    }

    thoht::SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    params.error_thresh = 0.5;

    LineEstimator line_estimator;
    Ransac<LineEstimator> ransac_line(params, line_estimator);
    ransac_line.Initialize();

    Line line;
    SacSummary summary;
    ransac_line.Estimate(input_points, &line, &summary);
    ASSERT_GE(summary.inliers.size(), 2500);
}

TEST(RandomSampler, UniqueMinimalSample)
{
    auto rng = std::make_shared<RandomNumberGenerator>(55);
    const int kMinNumSamples{3};
    const std::vector<int> data_points = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    RandomSampler sampler(rng, kMinNumSamples);
    CHECK(sampler.Initialize(data_points.size()));

    auto isUnique = [](const std::vector<int>& vec) -> bool {
        std::vector<int> sorted_vec = vec;
        std::sort(sorted_vec.begin(), sorted_vec.end());
        return std::unique(sorted_vec.begin(), sorted_vec.end()) ==
               sorted_vec.end();
    };

    for (int i = 0; i < 100; i++) {
        std::vector<int> subset;
        EXPECT_TRUE(sampler.Sample(&subset));

        // Make sure that the sampling is unique.
        EXPECT_EQ(subset.size(), kMinNumSamples);
        EXPECT_TRUE(isUnique(subset));
    }
}
