#include <gtest/gtest.h>

#include <tCore/RandomGenerator>
#include <tMath/Ransac/Lmed>

#include "test_utils.h"

using namespace tl;

namespace {

} // namespace

class LmedFitLine : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Prepare data
        constexpr size_t kNumInlier{5000};
        constexpr size_t kNumOutlier{2500};

        _points.clear();
        _points.reserve(kNumInlier + kNumOutlier);
        for (size_t i{0}; i < kNumInlier; ++i) {
            _points.emplace_back(i + _rng.randNorm(0., 0.1),
                                 i + _rng.randNorm(0., 0.1));
        }
        for (size_t i{0}; i < kNumOutlier; ++i) {
            _points.emplace_back(_rng.randFloat(0., 1e4),
                                 _rng.randFloat(0., 1e4));
        }

        std::random_device rd;
        std::mt19937 rng{rd()};
        std::shuffle(_points.begin(), _points.end(), rng);
    }

    void TearDown() override {}

    const auto& data() const { return _points; }

public:
    RandomNumberGenerator _rng{52};
    std::vector<Point> _points;
};

TEST_F(LmedFitLine, QualityMeasureOfCorrectModel)
{
    LineEstimator estimator;

    // y = 1
    const Line line{1., 0.};
    std::vector<double> residuals(data().size());
    for (size_t i{0}; i < residuals.size(); ++i) {
        residuals[i] = estimator.Error(data()[i], line);
    }

    LmedQualityMeasurement qa{estimator.SampleSize()};

    std::vector<size_t> inliers;
    EXPECT_LT(qa.ComputeCost(residuals, &inliers), 0.5);

    const double inlierRatio =
        static_cast<double>(inliers.size()) / residuals.size();
    EXPECT_NEAR(inlierRatio, 2. / 3., 0.1);
}

TEST_F(LmedFitLine, TerminationSlope)
{
    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(_rng);
    params.error_thresh = 5.0;

    LineEstimator estimator;
    LMed<LineEstimator> lmed{params, estimator};
    lmed.Initialize();

    Line line;
    SacSummary summary;
    CHECK(lmed.Estimate(data(), &line, &summary));

    EXPECT_LT(std::abs(line.m - 1.), 0.1);
    EXPECT_NEAR(static_cast<double>(summary.inliers.size()) / data().size(),
                2. / 3., 0.1);
}
