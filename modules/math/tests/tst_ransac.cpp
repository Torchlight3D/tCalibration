#include <numeric>

#include <gtest/gtest.h>

#include <tCore/RandomGenerator>
#include <tMath/Ransac/RANSAC>

#include "test_utils.h"

using namespace tl;

TEST(RANSAC, UniqueRandomSampler)
{
    // Data
    constexpr size_t kNumData{11};
    std::vector<int> data(kNumData);
    std::iota(data.begin(), data.end(), 0);

    // Sampler
    constexpr size_t kMinNumSamples{3};
    auto rng = std::make_shared<RandomNumberGenerator>(55);
    RandomSampler sampler{kMinNumSamples, rng};
    CHECK(sampler.Initialize(data.size()));

    auto isUnique = [](const std::vector<size_t>& vec) -> bool {
        auto sorted_vec = vec;
        std::sort(sorted_vec.begin(), sorted_vec.end());
        return std::unique(sorted_vec.begin(), sorted_vec.end()) ==
               sorted_vec.end();
    };

    constexpr auto kSampleCount = 100;
    for (auto i{0}; i < kSampleCount; ++i) {
        std::vector<size_t> subset;
        EXPECT_TRUE(sampler.Sample(&subset));

        // Make sure that the sampling is unique.
        EXPECT_EQ(subset.size(), kMinNumSamples);
        EXPECT_TRUE(isUnique(subset));
    }
}

class RANSACFitLine : public ::testing::Test
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
    RandomNumberGenerator _rng{46};
    std::vector<Point> _points;
};

TEST_F(RANSACFitLine, TerminationSlope)
{
    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(_rng);
    params.error_thresh = 0.5;

    LineEstimator estimator;
    RANSAC<LineEstimator> ransac{params, estimator};
    ransac.Initialize();

    Line line;
    SacSummary summary;
    CHECK(ransac.Estimate(_points, &line, &summary));
    ASSERT_LT(std::abs(line.m - 1.), 0.1);
}

TEST_F(RANSACFitLine, TerminationNumInliers)
{
    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(_rng);
    params.error_thresh = 0.5;

    LineEstimator estimator;
    RANSAC<LineEstimator> ransac{params, estimator};
    EXPECT_TRUE(ransac.Initialize());

    Line line;
    SacSummary summary;
    EXPECT_TRUE(ransac.Estimate(_points, &line, &summary));

    ASSERT_GE(summary.inliers.size(), 2500);
}
