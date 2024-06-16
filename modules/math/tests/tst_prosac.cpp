#include <gtest/gtest.h>

#include <tCore/RandomGenerator>
#include <tMath/Ransac/Prosac>

#include "test_utils.h"

using namespace tl;

class ProsacFitLine : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Prepare data
        // Create a set of points along y=x with a small random pertubation.
        constexpr size_t kNumPoint{10000};

        _points.clear();
        _confidences.clear();

        _points.reserve(kNumPoint);
        _confidences.reserve(kNumPoint);
        for (size_t i{0}; i < kNumPoint; ++i) {
            if (i < 300) {
                // Small noise
                _points.emplace_back(i + _rng.randNorm(0., 5e-2),
                                     i + _rng.randNorm(0., 5e-2));
                _confidences.emplace_back(0.95);
            }
            else {
                // Relative large noise
                _points.emplace_back(i + _rng.randNorm(0., 5e-1),
                                     i + _rng.randNorm(0., 5e-1));
                _confidences.emplace_back(0.1);
            }
        }
    }

protected:
    RandomNumberGenerator _rng{23};
    std::vector<Point> _points;
    std::vector<double> _confidences;
};

TEST_F(ProsacFitLine, TerminationSlope)
{
    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(_rng);
    params.error_thresh = 0.5;

    LineEstimator estimator;
    Prosac<LineEstimator> prosac{params, estimator};
    EXPECT_TRUE(prosac.Initialize());

    Line line;
    SacSummary summary;
    EXPECT_TRUE(prosac.Estimate(_points, &line, &summary));

    ASSERT_LT(std::abs(line.m - 1.0), 0.1);
}
