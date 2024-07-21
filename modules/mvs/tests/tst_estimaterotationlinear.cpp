#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Rotation>
#include <tMvs/Poses/EstimateRotationLinear>

#include "test_utils.h"

using namespace tl;

using Eigen::Vector3d;

namespace {
RandomNumberGenerator kRNG{56};
} // namespace

// FIXME: Some duplicated code exists
class EstimateRotationsLinear : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _opts.rotationNoise.reset();
        _orientations.clear();
        _viewPairs.clear();
    }

    void TearDown() override {}

    void Execute()
    {
        // Prepare data
        createOrientations(_opts.numViews);
        createViewPairs(_opts.numViewPairs, _opts.rotationNoise);

        // Estimate the rotations.
        LinearRotationEstimator estimator;

        // Set the initial rotation estimations.
        std::unordered_map<ViewId, Vector3d> rotations_est;
        EXPECT_TRUE(estimator.EstimateRotations(_viewPairs, &rotations_est));
        EXPECT_EQ(rotations_est.size(), _orientations.size());

        // Align the rotations and measure the error.
        AlignOrientations(_orientations, &rotations_est);
        for (const auto& [viewId, orientation] : _orientations) {
            const auto& rotation_est = con::FindOrDie(rotations_est, viewId);
            const auto relative_rotation =
                RelativeRotationFromTwoRotations(rotation_est, orientation);
            const auto error = math::radToDeg(relative_rotation.norm());

            EXPECT_LT(error, _ref.maxAngleDiff) << "\n"
                                                   "g.t. rotations: "
                                                << orientation.transpose()
                                                << "\n"
                                                   "estimated rotations = "
                                                << rotation_est.transpose();
        }
    }

protected:
    struct
    {
        int numViews = 10;
        int numViewPairs = 20;
        std::optional<double> rotationNoise = {}; // deg
    } _opts;

    struct
    {
        double maxAngleDiff = 1.; // deg
    } _ref;

private:
    void createOrientations(int numViews)
    {
        constexpr double kRotationScale = 0.2;
        for (int i = 0; i < numViews; i++) {
            _orientations[i] = kRotationScale * Vector3d::Random();
        }
    }

    void createViewPairs(size_t numViewPairs,
                         const std::optional<double>& noise)
    {
        // Create a set of view id pairs that will contain a spanning tree.
        for (size_t i = 1; i < _orientations.size(); i++) {
            const ViewIdPair viewPairId(i - 1, i);
            _viewPairs[viewPairId].rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(_orientations, viewPairId.first),
                con::FindOrDie(_orientations, viewPairId.second), noise);
        }

        // Add random edges.
        const ViewId maxViewId = static_cast<int>(_orientations.size()) - 1;
        while (_viewPairs.size() < numViewPairs) {
            const ViewIdPair viewIdPair{kRNG.randInt(0, maxViewId),
                                        kRNG.randInt(0, maxViewId)};

            if (!viewIdPair.isValid() || _viewPairs.contains(viewIdPair)) {
                continue;
            }

            _viewPairs[viewIdPair].rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(_orientations, viewIdPair.first),
                con::FindOrDie(_orientations, viewIdPair.second), noise);

            _viewPairs[viewIdPair].num_verified_matches = kRNG.randInt(50, 200);
        }
    }

private:
    std::unordered_map<ViewId, Eigen::Vector3d> _orientations;
    std::unordered_map<ViewIdPair, ViewPairInfo> _viewPairs;
};

TEST_F(EstimateRotationsLinear, SmallSceneNoNoise)
{
    _opts.numViews = 4;
    _opts.numViewPairs = 6;
    _ref.maxAngleDiff = 1e-6;

    Execute();
}

TEST_F(EstimateRotationsLinear, SmallSceneWithNoise)
{
    _opts.numViews = 4;
    _opts.numViewPairs = 6;
    _opts.rotationNoise = 1.;
    _ref.maxAngleDiff = 2.;

    Execute();
}

TEST_F(EstimateRotationsLinear, LargeSceneWithNoise)
{
    _opts.numViews = 100;
    _opts.numViewPairs = 800;
    _opts.rotationNoise = 5.;
    _ref.maxAngleDiff = 5.;

    Execute();
}
