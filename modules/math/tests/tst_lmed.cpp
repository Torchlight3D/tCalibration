#include <random>

#include <gtest/gtest.h>

#include <tCore/RandomGenerator>
#include <tMath/RANSAC/Lmed>

#include "test_utils.h"

using namespace tl;

namespace {
RandomNumberGenerator kRNG{52u};
} // namespace

class LmedTest : public ::testing::Test
{
public:
    static std::vector<Point>* input_points;

protected:
    void SetUp() override
    {
        constexpr int kNumInlierPoints{5000};
        constexpr int kNumOutlierPoints{2500};

        input_points = new std::vector<Point>;
        input_points->reserve(kNumInlierPoints + kNumOutlierPoints);
        for (int i{0}; i < kNumInlierPoints; ++i) {
            input_points->emplace_back(i + kRNG.RandGaussian(0.0, 0.1),
                                       i + kRNG.RandGaussian(0.0, 0.1));
        }
        for (int i{0}; i < kNumOutlierPoints; ++i) {
            input_points->emplace_back(kRNG.RandDouble(0.0, 10000),
                                       kRNG.RandDouble(0.0, 10000));
        }

        std::random_device rd;
        std::mt19937 g{rd()};

        std::shuffle(input_points->begin(), input_points->end(), g);
    }

    void TearDown() override { delete input_points; }
};

std::vector<Point>* LmedTest::input_points = nullptr;

// Tests the computation of the squared residuals by using the correct line
// model.
TEST_F(LmedTest, ComputingQualityMeasureOfCorrectModel)
{
    LineEstimator line_estimator;
    LmedQualityMeasurement lmed_quality_measurement(
        line_estimator.SampleSize());
    Line correct_line{1.0, 0.0};
    std::vector<double> residuals(input_points->size());
    for (int i = 0; i < residuals.size(); ++i) {
        residuals[i] = line_estimator.Error(input_points->at(i), correct_line);
    }

    std::vector<int> inliers;
    EXPECT_LT(lmed_quality_measurement.ComputeCost(residuals, &inliers), 0.5);
    const double inlier_ratio =
        static_cast<double>(inliers.size()) / residuals.size();
    EXPECT_NEAR(inlier_ratio, 0.666, 0.1);
}

// Tests the Lmed estimator by fitting a line to the input_points.
TEST_F(LmedTest, LineFitting)
{
    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(kRNG);
    // This threshold is arbitrary to comply with sample_consensus_estimator.h.
    params.error_thresh = 5.0;

    LineEstimator line_estimator;
    LMed<LineEstimator> lmed_line{params, line_estimator};
    lmed_line.Initialize();

    Line line;
    SacSummary summary;
    CHECK(lmed_line.Estimate(*input_points, &line, &summary));
    EXPECT_LT(fabs(line.m - 1.0), 0.1);
    EXPECT_NEAR(
        static_cast<double>(summary.inliers.size()) / input_points->size(),
        0.666, 0.1);
}
