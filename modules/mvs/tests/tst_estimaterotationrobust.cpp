#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Rotation>
#include <tMvs/Poses/EstimateRotationRobust>

#include "test_utils.h"

using namespace tl;

using Eigen::Vector3d;

namespace {
RandomNumberGenerator kRNG{56};
} // namespace

class EstimateRotationRobust : public ::testing::Test
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
        RobustRotationEstimator::Options opts;

        RobustRotationEstimator estimator{opts};

        // Set the initial rotation estimations.
        auto rotations_est = initRotationsFromSpanningTree();

        std::set<ViewId> fixedViewIds;
        for (int i = 0; i < _opts.numFixedView; ++i) {
            fixedViewIds.insert(i);
        }
        estimator.SetFixedGlobalRotations(fixedViewIds);

        EXPECT_TRUE(estimator.EstimateRotations(_viewPairs, &rotations_est));
        EXPECT_EQ(rotations_est.size(), _orientations.size());

        // Align the rotations and measure the error.
        AlignOrientations(_orientations, &rotations_est);
        for (const auto& [viewId, orientation] : _orientations) {
            const auto& rotation_est = con::FindOrDie(rotations_est, viewId);
            const auto relativeRotation =
                RelativeRotationFromTwoRotations(rotation_est, orientation);
            const auto angleDiff = math::radToDeg(relativeRotation.norm());

            EXPECT_LT(angleDiff, _ref.maxAngleDiff) << "\n"
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
        int numFixedView = 1;
        std::optional<double> rotationNoise = {}; // deg
    } _opts;

    struct
    {
        double maxAngleDiff = 1.; // deg
    } _ref;

private:
    void createOrientations(int numViews)
    {
        constexpr auto kRotationScale = 0.2;
        for (auto i{0}; i < numViews; i++) {
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
        }
    }

    std::unordered_map<ViewId, Eigen::Vector3d> initRotationsFromSpanningTree()
    {
        std::unordered_map<ViewId, Vector3d> rotations;
        rotations.reserve(_orientations.size());
        rotations.emplace(0, Vector3d::Zero());
        for (size_t i = 1; i < _orientations.size(); i++) {
            const auto id = static_cast<int>(i);
            rotations.emplace(id, ApplyRelativeRotation(
                                      con::FindOrDie(rotations, id - 1),
                                      con::FindOrDieNoPrint(
                                          _viewPairs, ViewIdPair{id - 1, id})
                                          .rotation));
        }

        return rotations;
    }

private:
    std::unordered_map<ViewId, Eigen::Vector3d> _orientations;
    std::unordered_map<ViewIdPair, ViewPairInfo> _viewPairs;
};

TEST_F(EstimateRotationRobust, SmallSceneNoNoise)
{
    _opts.numViews = 4;
    _opts.numViewPairs = 6;

    _ref.maxAngleDiff = 1e-8;

    Execute();
}

TEST_F(EstimateRotationRobust, SmallSceneWithNoise)
{
    _opts.numViews = 4;
    _opts.numViewPairs = 6;
    _opts.rotationNoise = 1.;

    _ref.maxAngleDiff = 1.;

    Execute();
}

TEST_F(EstimateRotationRobust, LargeSceneWithNoise)
{
    _opts.numViews = 100;
    _opts.numViewPairs = 800;
    _opts.rotationNoise = 2.;

    _ref.maxAngleDiff = 5.;

    Execute();
}

TEST_F(EstimateRotationRobust, SmallSceneNoNoiseFixedViews)
{
    _opts.numViews = 4;
    _opts.numViewPairs = 6;
    _opts.numFixedView = 2;
    _opts.rotationNoise = 2.;

    _ref.maxAngleDiff = 5.;

    Execute();
}

TEST_F(EstimateRotationRobust, LargeSceneWithNoiseFixedViews)
{
    _opts.numViews = 100;
    _opts.numViewPairs = 800;
    _opts.numFixedView = 5;
    _opts.rotationNoise = 2.;

    _ref.maxAngleDiff = 5.;

    Execute();
}
