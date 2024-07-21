#include <gtest/gtest.h>

#include <tCamera/Camera>
#include <tCore/ContainerUtils>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Rotation>
#include <tMvs/Scene>
#include <tMvs/Poses/EstimatePositionNonLinear>

#include "test_utils.h"

using namespace tl;

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
constexpr double kRelativeTranslationWeight = 1.0;

RandomNumberGenerator kRNG(63);

Camera RandomCamera()
{
    Camera camera;
    camera.setPosition(10. * Vector3d::Random());
    camera.setOrientationFromAngleAxis(0.2 * Vector3d::Random());
    camera.setImageSize(1000, 1000);
    camera.setFocalLength(800.);
    camera.setPrincipalPoint(500., 500.);
    return camera;
}

Camera SequentialCamera(const Eigen::Vector3d& dir, double stepSize)
{
    Camera camera;
    const Vector3d pos = dir * stepSize + Vector3d::Random() * 0.1;
    camera.setPosition(pos);
    camera.setOrientationFromAngleAxis(0.01 * Vector3d::Random());
    camera.setImageSize(1000, 1000);
    camera.setFocalLength(800.);
    camera.setPrincipalPoint(500., 500.);
    return camera;
}

} // namespace

class EstimatePositionsNonlinearTest : public ::testing::Test
{
protected:
    void SetUp() override {}

    void TestNonlinearPositionEstimator(
        int num_views, int num_tracks, int num_view_pairs, double pose_noise,
        double position_tolerance, std::set<ViewId> fixed_views,
        bool sequential_camera_trajectory = false, int min_nr_pts_per_view = 0,
        bool use_scale = false)
    {
        // Set up the camera.
        setupScene(num_views, num_tracks, sequential_camera_trajectory);
        setupViewPairs(num_view_pairs, pose_noise, use_scale);

        // Estimate the positions.
        NonlinearPositionEstimator::Options opts;
        opts.rng = std::make_shared<RandomNumberGenerator>(kRNG);
        opts.min_num_points_per_view = min_nr_pts_per_view;

        NonlinearPositionEstimator estimator{opts, _scene};

        std::unordered_map<ViewId, Vector3d> positions_est;
        if (!fixed_views.size()) {
            EXPECT_TRUE(estimator.EstimatePositions(_viewPairs, _orientations,
                                                    &positions_est));
            EXPECT_EQ(positions_est.size(), _positions.size());
        }
        else {
            // Use all view ids
            const auto viewIds = _scene.viewIds();
            const std::unordered_set<ViewId> subViewIds{viewIds.cbegin(),
                                                        viewIds.cend()};
            // set positions of fixed views
            for (const auto& viewId : fixed_views) {
                positions_est.emplace(viewId,
                                      _scene.view(viewId)->camera().position());
            }

            estimator.EstimateRemainingPositionsInRecon(
                fixed_views, subViewIds, _viewPairs, &positions_est);
        }

        // Align the positions and measure the error.
        AlignPositions(_positions, &positions_est);
        for (const auto& [viewId, position] : _positions) {
            const auto& position_est = con::FindOrDie(positions_est, viewId);
            const double position_error =
                (position - position_est).squaredNorm();

            EXPECT_LT(position_error, position_tolerance)
                << "\nview id = " << viewId
                << "\ng.t. position = " << position.transpose()
                << "\nestimated position = " << position_est.transpose();
        }
    }

protected:
    struct
    {
        int numViews = 10;
        int numTracks = 10;
        std::optional<double> poseNoise = {}; // deg
    } _opts;

private:
    void setupScene(int numViews, int numTracks, bool sequentialCamera = false)
    {
        // Create random views.
        std::vector<ViewId> viewIds;
        for (int i{0}; i < numViews; ++i) {
            const auto i_float = static_cast<double>(i);

            const ViewId viewId = _scene.addView(std::to_string(i), i_float);
            viewIds.push_back(viewId);

            // Create a random pose.
            if (sequentialCamera) {
                _scene.rView(viewId)->rCamera() =
                    SequentialCamera(Vector3d{0., i_float, 0.}, 1.);
            }
            else {
                _scene.rView(viewId)->rCamera() = RandomCamera();
            }

            _orientations[viewId] =
                _scene.view(viewId)->camera().orientationAsAngleAxis();
            _positions[viewId] = _scene.view(viewId)->camera().position();
        }

        // Add random tracks.
        std::random_device rd;
        std::mt19937 rng{rd()};

        for (int i = 0; i < numTracks; i++) {
            // Shuffle the view ids so that we can obtain tracks in random
            // views.
            std::shuffle(viewIds.begin(), viewIds.end(), rng);

            // Create a track that is seen in several views.
            Vector4d point = Vector4d::Random();
            point[2] += 20.;
            point[3] = 1.;

            Scene::TrackObservation observations;
            for (const auto& viewId : viewIds) {
                const auto* view = _scene.view(viewId);
                Vector2d pixel;
                view->camera().projectPoint(point, pixel);
                observations.insert({viewId, pixel});
            }

            auto* track = _scene.rTrack(_scene.addTrack(observations));
            track->setEstimated(true);
            track->rPosition() = point;
        }
    }

    void setupViewPairs(size_t numViewPairs,
                        const std::optional<double>& noise = {},
                        bool useScale = false)
    {
        // Create a single connected component.
        std::vector<ViewId> viewIds;
        viewIds.push_back(0);
        for (size_t i{1}; i < _positions.size(); ++i) {
            const auto viewId_i = static_cast<int>(i);
            const ViewIdPair viewPairId{viewId_i - 1, viewId_i};
            _viewPairs[viewPairId] =
                createViewPairInfo(viewPairId, noise, useScale);
            viewIds.push_back(viewId_i);
        }

        std::random_device rd;
        std::mt19937 rng{rd()};
        while (_viewPairs.size() < numViewPairs) {
            std::shuffle(viewIds.begin(), viewIds.end(), rng);
            const ViewIdPair viewPairId{viewIds[0], viewIds[1]};
            if (_viewPairs.contains(viewPairId)) {
                continue;
            }

            _viewPairs[viewPairId] =
                createViewPairInfo(viewPairId, noise, useScale);
        }
    }

    ViewPairInfo createViewPairInfo(const ViewIdPair& viewPairId,
                                    const std::optional<double>& noise = {},
                                    bool useScale = false) const
    {
        const auto& viewId1 = viewPairId.first;
        const auto& viewId2 = viewPairId.second;

        ViewPairInfo info;
        info.focalLength1 = 800.;
        info.focalLength2 = 800.;

        if (noise.has_value()) {
            const Vector2d extra = noise.value() * Vector2d::Random();

            // Determine the relative rotation and add noise.
            info.rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(_orientations, viewId1),
                con::FindOrDie(_orientations, viewId2), extra(0));

            // Determine the relative position and add noise.
            const auto& position1 = con::FindOrDie(_positions, viewId1);
            const auto& position2 = con::FindOrDie(_positions, viewId2);
            info.scale_estimate = (position2 - position1).norm();

            const auto rotation1 = con::FindOrDie(_orientations, viewId1);
            info.position = RelativeTranslationFromTwoPositions(
                position1, position2, rotation1, extra(1));
        }
        else {
            // Determine the relative rotation and add noise.
            info.rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(_orientations, viewId1),
                con::FindOrDie(_orientations, viewId2));

            // Determine the relative position and add noise.
            const auto& position1 = con::FindOrDie(_positions, viewId1);
            const auto& position2 = con::FindOrDie(_positions, viewId2);
            info.scale_estimate = (position2 - position1).norm();

            const auto rotation1 = con::FindOrDie(_orientations, viewId1);
            info.position = RelativeTranslationFromTwoPositions(
                position1, position2, rotation1);
        }

        if (useScale) {
            if (noise.has_value()) {
                info.scale_estimate +=
                    kRNG.randFloat<double>() * noise.value() / 2.;
            }
        }
        else {
            info.scale_estimate = -1.;
        }

        return info;
    }

private:
    std::unordered_map<ViewId, Eigen::Vector3d> _positions;
    std::unordered_map<ViewId, Eigen::Vector3d> _orientations;
    std::unordered_map<ViewIdPair, ViewPairInfo> _viewPairs;
    Scene _scene;
};

TEST_F(EstimatePositionsNonlinearTest, SmallSceneNoNoiseNoScale)
{
    constexpr double kTolerance = 1e-4;
    constexpr int kNumViews = 4;
    constexpr int kNumTracksPerView = 10;
    constexpr int kNumViewPairs = 6;
    std::set<ViewId> fixed_views;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   0.0, kTolerance, fixed_views);
}

TEST_F(EstimatePositionsNonlinearTest, SmallSceneNoNoiseWithScale)
{
    constexpr double kTolerance = 1e-4;
    constexpr int kNumViews = 4;
    constexpr int kNumTracksPerView = 10;
    constexpr int kNumViewPairs = 6;
    constexpr bool kUseScale = true;
    std::set<ViewId> fixed_views;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   0.0, kTolerance, fixed_views, false, 0,
                                   kUseScale);
}

TEST_F(EstimatePositionsNonlinearTest, SmallSceneWithNoiseNoScale)
{
    static const double kTolerance = 0.1;
    static const int kNumViews = 4;
    static const int kNumTracksPerView = 10;
    static const int kNumViewPairs = 6;
    static const double kPoseNoiseDegrees = 1.0;
    std::set<ViewId> fixed_views;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   kPoseNoiseDegrees, kTolerance, fixed_views);
}

TEST_F(EstimatePositionsNonlinearTest, SmallSceneWithNoiseWithScale)
{
    static const double kTolerance = 0.1;
    static const int kNumViews = 4;
    static const int kNumTracksPerView = 10;
    static const int kNumViewPairs = 6;
    static const double kPoseNoiseDegrees = 1.0;
    static const bool kUseScale = true;
    std::set<ViewId> fixed_views;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   kPoseNoiseDegrees, kTolerance, fixed_views,
                                   false, 0, kUseScale);
}

TEST_F(EstimatePositionsNonlinearTest,
       SmallSceneNoNoiseNoScaleFixedCamsSequential)
{
    static const double kTolerance = 0.1;
    static const int kNumViews = 4;
    static const int kNumTracksPerView = 10;
    static const int kNumViewPairs = 6;
    static const double kPoseNoiseDegrees = 0.0;
    std::set<ViewId> fixed_views = {0};
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   kPoseNoiseDegrees, kTolerance, fixed_views,
                                   true);
}

TEST_F(EstimatePositionsNonlinearTest,
       SmallSceneWithNoiseWithScaleCamsSequential)
{
    static const double kTolerance = 0.1;
    static const int kNumViews = 4;
    static const int kNumTracksPerView = 10;
    static const int kNumViewPairs = 6;
    static const double kPoseNoiseDegrees = 0.1;
    static const bool kUseScale = true;
    std::set<ViewId> fixed_views;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   kPoseNoiseDegrees, kTolerance, fixed_views,
                                   true, 0, kUseScale);
}

TEST_F(EstimatePositionsNonlinearTest, SmallTestNoiseFixedCamsSequential)
{
    static const double kTolerance = 0.1;
    static const int kNumViews = 4;
    static const int kNumTracksPerView = 10;
    static const int kNumViewPairs = 6;
    static const double kPoseNoiseDegrees = 1.0;
    std::set<ViewId> fixed_views = {0, 1};
    static const int kNrPointsPerView = 5;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   kPoseNoiseDegrees, kTolerance, fixed_views,
                                   true, kNrPointsPerView);
}

TEST_F(EstimatePositionsNonlinearTest, LargeTestNoiseFixedCamsSequential)
{
    static const double kTolerance = 0.1;
    static const int kNumViews = 20;
    static const int kNumTracksPerView = 10;
    static const int kNumViewPairs = 50;
    static const double kPoseNoiseDegrees = 0.5;
    std::set<ViewId> fixed_views = {0, 1, 2, 3, 4, 5};
    static const int kNrPointsPerView = 10;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   kPoseNoiseDegrees, kTolerance, fixed_views,
                                   true, kNrPointsPerView);
}

TEST_F(EstimatePositionsNonlinearTest, LargeTestNoiseWScale)
{
    GTEST_SKIP()
        << "This test case takes a while to run. Enable if neccessary.";

    static const double kTolerance = 0.5;
    static const int kNumViews = 1000;
    static const int kNumTracksPerView = 10;
    static const int kNumViewPairs = 3000;
    static const double kPoseNoiseDegrees = 0.5;
    static const bool kUseScale = true;
    std::set<ViewId> fixed_views;
    static const int kNrPointsPerView = 10;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   kPoseNoiseDegrees, kTolerance, fixed_views,
                                   false, kNrPointsPerView, kUseScale);
}

TEST_F(EstimatePositionsNonlinearTest, LargeTestNoiseFixedCamsSequentialWScale)
{
    GTEST_SKIP()
        << "This test case takes a while to run. Enable if neccessary.";

    static const double kTolerance = 0.5;
    static const int kNumViews = 1000;
    static const int kNumTracksPerView = 10;
    static const int kNumViewPairs = 3000;
    static const double kPoseNoiseDegrees = 0.5;
    static const bool kUseScale = true;
    std::set<ViewId> fixed_views = {0, 1, 2, 3, 4, 5};
    static const int kNrPointsPerView = 10;
    TestNonlinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                   kPoseNoiseDegrees, kTolerance, fixed_views,
                                   true, kNrPointsPerView, kUseScale);
}

void PairwiseTranslationErrorTest(const Eigen::Vector3d& known_translation,
                                  double weight,
                                  const Eigen::Vector3d& position_1,
                                  const Eigen::Vector3d& position_2)
{
    // Compute ground truth angular error.
    Vector3d translation = position_2 - position_1;
    static const double kEpsilon = 1e-8;
    if (translation.norm() > kEpsilon) {
        translation.normalize();
    }

    const Vector3d expected_error = weight * (translation - known_translation);

    // Initialize error function and compute rotation error.
    const PairwiseTranslationError translation_error(known_translation, weight);
    Vector3d error = Vector3d::Zero();
    translation_error(position_1.data(), position_2.data(), error.data());

    EXPECT_DOUBLE_EQ(error(0), expected_error(0));
    EXPECT_DOUBLE_EQ(error(1), expected_error(1));
    EXPECT_DOUBLE_EQ(error(2), expected_error(2));
}

TEST(PairwiseTranslationErrorT, NoTranslation)
{
    const Vector3d position_1(1.0, 0.0, 0.0);
    const Vector3d position_2(1.0, 0.0, 0.0);
    const Vector3d relative_translation(0.0, 0.0, 0.0);

    // Initialize error function and compute rotation error.
    const PairwiseTranslationError translation_error(relative_translation, 1.0);
    Vector3d error = Vector3d::Zero();
    EXPECT_TRUE(
        translation_error(position_1.data(), position_2.data(), error.data()));
}

TEST(PairwiseTranslationErrorT, TranslationNoNoise)
{
    const Vector3d position_1(0.0, 0.0, 0.0);
    const Vector3d position_2(1.0, 0.0, 0.0);
    const Vector3d relative_translation =
        (position_2 - position_1).normalized();

    PairwiseTranslationErrorTest(relative_translation,
                                 kRelativeTranslationWeight, position_1,
                                 position_2);
}

TEST(PairwiseTranslationErrorT, TranslationWithNoise)
{
    const Vector3d position_1(0.0, 0.0, 0.0);
    const Vector3d position_2(1.0, 0.0, 0.0);
    Vector3d relative_translation = (position_2 - position_1).normalized();

    // Add noise.
    relative_translation =
        (relative_translation + Vector3d(0.01, 0.01, 0.01)).normalized();

    PairwiseTranslationErrorTest(relative_translation,
                                 kRelativeTranslationWeight, position_1,
                                 position_2);
}

TEST(PairwiseTranslationErrorT, NontrivialWeight)
{
    const Vector3d position_1(0.0, 0.0, 0.0);
    const Vector3d position_2(1.0, 0.0, 0.0);
    Vector3d relative_translation = (position_2 - position_1).normalized();

    // Add noise.
    relative_translation =
        (relative_translation + Vector3d(0.01, 0.01, 0.01)).normalized();

    constexpr double kNontrivialWeight = 1.1;
    PairwiseTranslationErrorTest(relative_translation, kNontrivialWeight,
                                 position_1, position_2);
}
