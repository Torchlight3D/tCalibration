#include <random>

#include <gtest/gtest.h>

#include <tMath/RANSAC/Prosac>

#include "test_utils.h"

using namespace tl;

TEST(ProsacTest, LineFitting)
{
    // TODO:
    // 1. use AxMath/RandomGenerator
    // 2. use Fixture

    // Create a set of points along y=x with a small random pertubation.
    std::default_random_engine generator(90);
    std::normal_distribution<double> small_distribution(0.0, 0.05);
    std::normal_distribution<double> gauss_distribution(0.0, 0.5);
    constexpr int num_points = 10000;
    std::vector<Point> input_points(num_points);
    std::vector<double> confidence(num_points);

    for (int i = 0; i < num_points; ++i) {
        if (i < 300) {
            double noise_x = small_distribution(generator);
            double noise_y = small_distribution(generator);
            input_points[i] = Point(i + noise_x, i + noise_y);
            confidence[i] = 0.95;
        }
        else {
            double noise_x = gauss_distribution(generator);
            double noise_y = gauss_distribution(generator);
            input_points[i] = Point(i + noise_x, i + noise_y);
            confidence[i] = 0.1;
        }
    }

    SacParameters params;
    params.rng = std::make_shared<RandomNumberGenerator>(113);
    params.error_thresh = 0.5;

    LineEstimator line_estimator;
    Prosac<LineEstimator> prosac_line{params, line_estimator};
    prosac_line.Initialize();

    Line line;
    SacSummary summary;
    prosac_line.Estimate(input_points, &line, &summary);
    ASSERT_LT(fabs(line.m - 1.0), 0.1);
}
