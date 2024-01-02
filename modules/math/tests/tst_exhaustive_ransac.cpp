#include <numeric>

#include <gtest/gtest.h>

#include <AxMath/RandomGenerator>
#include <AxMath/ExhaustiveRansac>

#include "test_utils.h"

using namespace thoht;

TEST(ExhaustiveSampler, EnsureExhaustiveSample)
{
    constexpr int kNumDataPoints{100};
    std::vector<int> data(kNumDataPoints);
    std::iota(data.begin(), data.end(), 0);

    auto rng = std::make_shared<RandomNumberGenerator>(55);
    constexpr int kMinNumSamples{2};
    ExhaustiveSampler sampler{rng, kMinNumSamples};
    CHECK(sampler.Initialize(data.size()));

    for (int i = 0; i < data.size(); i++) {
        for (int j = i + 1; j < data.size(); j++) {
            std::vector<int> subset;
            EXPECT_TRUE(sampler.Sample(&subset));

            // Make sure that the sampling is unique.
            EXPECT_EQ(subset.size(), kMinNumSamples);
            EXPECT_EQ(subset[0], i);
            EXPECT_EQ(subset[1], j);
        }
    }

    // The next sample after all combinations are enumerated should be (0, 1).
    std::vector<int> subset;
    EXPECT_TRUE(sampler.Sample(&subset));
    EXPECT_EQ(subset[0], 0);
    EXPECT_EQ(subset[1], 1);
}

namespace {
RandomNumberGenerator kRng(46);
} // namespace

TEST(ExhaustiveRansacTest, LineFitting)
{
    // Create a set of points along y=x with a small random pertubation.
    constexpr size_t kDataPointSize{10000};
    std::vector<Point> points;
    points.reserve(kDataPointSize);
    for (size_t i{0}; i < kDataPointSize; ++i) {
        if (i % 2 == 0) {
            double noise_x = kRng.RandGaussian(0.0, 0.1);
            double noise_y = kRng.RandGaussian(0.0, 0.1);
            points.emplace_back(i + noise_x, i + noise_y);
        }
        else {
            double noise_x = kRng.RandDouble(0.0, 10000);
            double noise_y = kRng.RandDouble(0.0, 10000);
            points.emplace_back(noise_x, noise_y);
        }
    }

    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(kRng);
    params.error_thresh = 0.5;

    LineEstimator line_estimator;
    ExhaustiveRansac<LineEstimator> ransac_line{params, line_estimator};
    ransac_line.Initialize();

    Line line;
    SacSummary summary;
    CHECK(ransac_line.Estimate(points, &line, &summary));
    ASSERT_LT(fabs(line.m - 1.0), 0.1);
}

TEST(ExhaustiveRansacTest, TerminationNumInliers)
{
    // Create a set of points along y=x with a small random pertubation.
    constexpr size_t kDataPointSize{10000};
    std::vector<Point> input_points;
    input_points.reserve(kDataPointSize);
    for (int i = 0; i < 10000; ++i) {
        if (i % 2 == 0) {
            double noise_x = kRng.RandGaussian(0., 0.1);
            double noise_y = kRng.RandGaussian(0., 0.1);
            input_points.emplace_back(i + noise_x, i + noise_y);
        }
        else {
            double noise_x = kRng.RandDouble(0., 10000);
            double noise_y = kRng.RandDouble(0., 10000);
            input_points.emplace_back(noise_x, noise_y);
        }
    }

    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(kRng);
    params.error_thresh = 0.5;

    LineEstimator line_estimator;
    ExhaustiveRansac<LineEstimator> ransac_line(params, line_estimator);
    ransac_line.Initialize();

    Line line;
    SacSummary summary;
    ransac_line.Estimate(input_points, &line, &summary);
    ASSERT_GE(summary.inliers.size(), 2500);
}
