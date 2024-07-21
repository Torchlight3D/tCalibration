#include <gtest/gtest.h>

#include <tCamera/Camera>
#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Rotation>
#include <tMvs/Scene>
#include <tMvs/Poses/EstimatePositionLiGT>

#include "test_utils.h"

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
RandomNumberGenerator kRNG{42};

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

} // namespace

class EstimatePositionsLiGT : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Configs
        _opts.imageNoise.reset();
        _opts.poseNoise.reset();

        // Data
        _orientations.reserve(_opts.numViews);
        _orientations.clear();
        _positions.reserve(_opts.numViews);
        _positions.clear();
    }

    void TearDown() override {}

    void Execute()
    {
        // Prepare data
        setupScene(_opts.numViews, _opts.numTracksPerView, _opts.imageNoise);
        setupViewPairs(_opts.numViewPairs, _opts.poseNoise);

        // Estimate the positions.
        LiGTPositionEstimator::Options opts;

        LiGTPositionEstimator estimator{opts, _scene};

        std::unordered_map<ViewId, Vector3d> positions_est;
        EXPECT_TRUE(estimator.EstimatePositions(_viewPairs, _orientations,
                                                &positions_est));
        EXPECT_EQ(positions_est.size(), _positions.size());

        // Align the positions and measure the error.
        AlignPositions(_positions, &positions_est);

        for (const auto& [viewId, position] : _positions) {
            const auto& position_est = con::FindOrDie(positions_est, viewId);
            const double positionDiff = (position - position_est).norm();
            EXPECT_LT(positionDiff, _ref.maxPositionDiff)
                << "\n"
                   "Ground truth position: "
                << position.transpose()
                << "\n"
                   "Estimated position: "
                << position_est.transpose();
        }
    }

protected:
    struct
    {
        int numViews = 10;
        int numTracksPerView = 64;
        int numViewPairs = 10;
        std::optional<double> imageNoise = {};
        std::optional<double> poseNoise = {};
    } _opts;

    struct
    {
        double maxPositionDiff = 1.;
    } _ref;

private:
    void setupScene(int numViews, int numTracksPerView,
                    const std::optional<double>& noise)
    {
        // Create views with random pose
        std::vector<ViewId> viewIds;
        for (int i{0}; i < numViews; ++i) {
            const auto viewId =
                _scene.addView(std::to_string(i), 0., CameraId{i});
            viewIds.push_back(viewId);

            // Create a random pose.
            _scene.rView(viewId)->rCamera() = RandomCamera();

            const auto& cam = _scene.view(viewId)->camera();
            _orientations[viewId] = cam.orientationAsAngleAxis();
            _positions[viewId] = cam.position();
        }

        // Add random tracks.
        std::random_device rd;
        std::mt19937 rng{rd()};
        for (int i{0}; i < numTracksPerView; i++) {
            std::shuffle(viewIds.begin(), viewIds.end(), rng);

            // Create a track that is seen in several views.
            Vector4d point = Vector4d::Random();
            point[2] += 20.; // [19, 21]m depth
            point[3] = 1.;   // Homo

            Scene::TrackObservation obs;
            obs.reserve(viewIds.size());
            for (const auto& viewId : viewIds) {
                const auto* view = _scene.view(viewId);
                Vector2d pixel;
                [[maybe_unused]] const auto depth =
                    view->camera().projectPoint(point, pixel);
                if (noise.has_value()) {
                    AddNoiseToVector2(noise.value(), &pixel);
                }
                obs.insert({viewId, pixel});
            }

            _scene.rTrack(_scene.addTrack(obs))->rPosition() = point;
        }
    }

    void setupViewPairs(int numViewPairs, const std::optional<double>& noise)
    {
        // Create a single connected component.
        std::vector<ViewId> viewIds;
        viewIds.emplace_back(0);
        for (size_t i{1}; i < _positions.size(); ++i) {
            const ViewIdPair viewIdPair{static_cast<ViewId>(i - 1),
                                        static_cast<ViewId>(i)};
            _viewPairs[viewIdPair] = createViewPairInfo(viewIdPair, noise);
            viewIds.push_back(i);
        }

        std::random_device rd;
        std::mt19937 rng{rd()};
        while (static_cast<int>(_viewPairs.size()) < numViewPairs) {
            std::shuffle(viewIds.begin(), viewIds.end(), rng);
            const ViewIdPair viewIdPair{viewIds[0], viewIds[1]};
            if (_viewPairs.contains(viewIdPair)) {
                continue;
            }

            _viewPairs[viewIdPair] = createViewPairInfo(viewIdPair, noise);
        }
    }

    ViewPairInfo createViewPairInfo(const ViewIdPair& viewIdPair,
                                    const std::optional<double>& noise) const
    {
        const auto& viewId1 = viewIdPair.first;
        const auto& viewId2 = viewIdPair.second;

        ViewPairInfo info;
        info.focalLength1 = 800.;
        info.focalLength2 = 800.;

        if (noise.has_value()) {
            // These objects will add extra to the relative pose.
            const Vector2d extra = noise.value() * Vector2d::Random();

            // Determine the relative rotation and add noise.
            info.rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(_orientations, viewId1),
                con::FindOrDie(_orientations, viewId2), extra(0));

            // Determine the relative position and add noise.
            info.position = RelativeTranslationFromTwoPositions(
                con::FindOrDie(_positions, viewId1),
                con::FindOrDie(_positions, viewId2),
                con::FindOrDie(_orientations, viewId1), extra(1));
        }
        else {
            // Determine the relative rotation and add noise.
            info.rotation = RelativeRotationFromTwoRotations(
                con::FindOrDie(_orientations, viewId1),
                con::FindOrDie(_orientations, viewId2));

            // Determine the relative position and add noise.
            info.position = RelativeTranslationFromTwoPositions(
                con::FindOrDie(_positions, viewId1),
                con::FindOrDie(_positions, viewId2),
                con::FindOrDie(_orientations, viewId1));
        }

        return info;
    }

private:
    std::unordered_map<ViewId, Eigen::Vector3d> _positions;
    std::unordered_map<ViewId, Eigen::Vector3d> _orientations;
    std::unordered_map<ViewIdPair, ViewPairInfo> _viewPairs;
    Scene _scene;
};

TEST_F(EstimatePositionsLiGT, MinimumSceneNoNoise)
{
    _opts.numViews = 3;
    _opts.numTracksPerView = 5;
    _opts.numViewPairs = 3;

    _ref.maxPositionDiff = 1e-4;

    Execute();
}

// FIXME: This test case would occasionally fail
TEST_F(EstimatePositionsLiGT, MinimumSceneWithNoise)
{
    _opts.numViews = 3;
    _opts.numTracksPerView = 5;
    _opts.numViewPairs = 3;
    _opts.poseNoise = 1.;
    _opts.imageNoise = 0.5;

    _ref.maxPositionDiff = 0.5;

    Execute();
}

TEST_F(EstimatePositionsLiGT, SmallSceneNoNoise)
{
    _opts.numViews = 200;
    _opts.numTracksPerView = 100;
    _opts.numViewPairs = 500;

    _ref.maxPositionDiff = 1e-4;

    Execute();
}

TEST_F(EstimatePositionsLiGT, SmallSceneWithNoise)
{
    _opts.numViews = 200;
    _opts.numTracksPerView = 100;
    _opts.numViewPairs = 500;
    _opts.poseNoise = 1.;
    _opts.imageNoise = 0.5;

    _ref.maxPositionDiff = 0.5;

    Execute();
}

TEST_F(EstimatePositionsLiGT, LargeSceneWithNoise)
{
    GTEST_SKIP()
        << "This test case takes a while to run. Enable when neccessary.";

    _opts.numViews = 2000;
    _opts.numTracksPerView = 150;
    _opts.numViewPairs = 5000;
    _opts.poseNoise = 1.;
    _opts.imageNoise = 0.5;

    _ref.maxPositionDiff = 0.5;

    Execute();
}
