#include <numeric>

#include <gtest/gtest.h>

#include <tCore/RandomGenerator>
#include <tMath/Ransac/ExhaustiveRansac>

#include "test_utils.h"

using namespace tl;

TEST(ExhaustiveRansac, UniqueExhaustiveSample)
{
    // Data
    constexpr size_t kNumData{100};
    std::vector<int> data(kNumData);
    std::iota(data.begin(), data.end(), 0);

    // Sampler
    constexpr size_t kMinNumSamples{2};
    auto rng = std::make_shared<RandomNumberGenerator>(55);
    ExhaustiveSampler sampler{kMinNumSamples, rng};
    CHECK(sampler.Initialize(data.size()));

    for (size_t i{0}; i < data.size(); ++i) {
        for (size_t j = i + 1; j < data.size(); ++j) {
            std::vector<size_t> subset;
            EXPECT_TRUE(sampler.Sample(&subset));

            // Check every sample is unique
            EXPECT_EQ(subset.size(), kMinNumSamples);
            EXPECT_EQ(subset[0], i);
            EXPECT_EQ(subset[1], j);
        }
    }

    // Check the next sample after all combinations should be (0, 1).
    std::vector<size_t> subset;
    EXPECT_TRUE(sampler.Sample(&subset));
    EXPECT_EQ(subset[0], 0);
    EXPECT_EQ(subset[1], 1);
}

class ExhaustiveRansacFitLine : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Prepare data
        // Create a set of points along y=x with a small noise.
        constexpr size_t kPointCount{10000};

        _points.clear();
        _points.reserve(kPointCount);
        for (size_t i{0}; i < kPointCount; ++i) {
            if (i % 2 == 0) {
                _points.emplace_back(i + _rng.randNorm(0., 0.1),
                                     i + _rng.randNorm(0., 0.1));
            }
            else {
                _points.emplace_back(_rng.randFloat(0., 1e4),
                                     _rng.randFloat(0., 1e4));
            }
        }
    }

protected:
    RandomNumberGenerator _rng{56756};
    std::vector<Point> _points;
};

TEST_F(ExhaustiveRansacFitLine, TerminationSlope)
{
    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(_rng);
    params.error_thresh = 0.5;

    LineEstimator estimator;
    ExhaustiveRansac<LineEstimator> ransac{params, estimator};
    CHECK(ransac.Initialize());

    Line line;
    SacSummary summary;
    CHECK(ransac.Estimate(_points, &line, &summary));

    ASSERT_LT(std::abs(line.m - 1.), 0.1);
}

TEST_F(ExhaustiveRansacFitLine, TerminationNumInliers)
{
    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(_rng);
    params.error_thresh = 0.5;

    LineEstimator estimator;
    ExhaustiveRansac<LineEstimator> ransac{params, estimator};
    CHECK(ransac.Initialize());

    Line line;
    SacSummary summary;
    CHECK(ransac.Estimate(_points, &line, &summary));

    // FIXME: Fail in some random data (like when seed = 46, )
    ASSERT_GE(summary.inliers.size(), 2500);
}
