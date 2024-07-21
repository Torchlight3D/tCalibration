#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <tCamera/Camera>
#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Rotation>
#include <tMvs/Scene>
#include <tMvs/Poses/EstimatePositionLinear>

#include "test_utils.h"

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {

RandomNumberGenerator kRNG(62);

Camera RandomCamera()
{
    Camera camera;
    camera.setPosition(10 * Vector3d::Random());
    camera.setOrientationFromAngleAxis(0.2 * Vector3d::Random());
    camera.setImageSize(1000, 1000);
    camera.setFocalLength(800);
    camera.setPrincipalPoint(500.0, 500.0);
    return camera;
}

Vector3d RelativeRotationFromTwoRotations(const Vector3d& rotation1,
                                          const Vector3d& rotation2,
                                          const double noise)
{
    const Eigen::Matrix3d noisy_rotation =
        Eigen::AngleAxisd(math::degToRad(noise),
                          Vector3d::Random().normalized())
            .toRotationMatrix();

    Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
    ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
    ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

    const Eigen::AngleAxisd relative_rotation(
        noisy_rotation * rotation_matrix2 * rotation_matrix1.transpose());
    return relative_rotation.angle() * relative_rotation.axis();
}

Vector3d RelativeTranslationFromTwoPositions(const Vector3d& position1,
                                             const Vector3d& position2,
                                             const Vector3d& rotation1,
                                             const double noise)
{
    const Eigen::AngleAxisd noisy_translation(math::degToRad(noise),
                                              Vector3d::Random().normalized());
    Eigen::Matrix3d rotation_matrix1;
    ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
    const Vector3d relative_translation =
        rotation_matrix1 * (position2 - position1).normalized();
    return noisy_translation * relative_translation;
}

} // namespace

class EstimatePositionsLinearTest : public ::testing::Test
{
public:
    void TestLinearPositionEstimator(const int num_views, const int num_tracks,
                                     const int num_view_pairs,
                                     const double pose_noise,
                                     const double position_tolerance)
    {
        // Set up the camera.
        SetupReconstruction(num_views, num_tracks);
        GetTwoViewInfos(num_view_pairs, pose_noise);

        // Estimate the positions.
        LinearPositionEstimator position_estimator(options_, reconstruction_);

        std::unordered_map<ViewId, Vector3d> estimated_positions;
        EXPECT_TRUE(position_estimator.EstimatePositions(
            view_pairs_, orientations_, &estimated_positions));
        EXPECT_EQ(estimated_positions.size(), positions_.size());

        // Align the positions and measure the error.
        AlignPositions(positions_, &estimated_positions);
        for (const auto& position : positions_) {
            const Vector3d& estimated_position =
                con::FindOrDie(estimated_positions, position.first);
            const double position_error =
                (position.second - estimated_position).norm();
            EXPECT_LT(position_error, position_tolerance)
                << "\ng.t. position = " << position.second.transpose()
                << "\nestimated position = " << estimated_position.transpose();
        }
    }

protected:
    void SetUp() {}

    void SetupReconstruction(const int num_views, const int num_tracks)
    {
        // Create random views.
        std::vector<ViewId> view_ids;
        for (int i = 0; i < num_views; i++) {
            const ViewId view_id =
                reconstruction_.addView(std::to_string(i), 0, i);
            view_ids.push_back(view_id);

            // Create a random pose.
            reconstruction_.rView(view_id)->rCamera() = RandomCamera();
            orientations_[view_id] = reconstruction_.view(view_id)
                                         ->camera()
                                         .orientationAsAngleAxis();
            positions_[view_id] =
                reconstruction_.view(view_id)->camera().position();
        }

        std::random_device rd;
        std::mt19937_64 rngg{rd()};
        // Add random tracks.
        for (int i = 0; i < num_tracks; i++) {
            // Shuffle the view ids so that we can obtain tracks in random
            // views.
            std::shuffle(view_ids.begin(), view_ids.end(), rngg);

            // Create a track that is seen in several views.
            Vector4d point = Vector4d::Random();
            point[2] += 20.0;
            point[3] = 1.0;
            Scene::TrackObservation features;
            for (size_t j = 0; j < view_ids.size(); j++) {
                const View* view = reconstruction_.view(view_ids[j]);
                Eigen::Vector2d pixel;
                view->camera().projectPoint(point, pixel);
                features.emplace(view_ids[j], pixel);
            }
            Track* track =
                reconstruction_.rTrack(reconstruction_.addTrack(features));
            track->rPosition() = point;
        }
    }

    void GetTwoViewInfos(const size_t num_view_pairs, const double pose_noise)
    {
        // Create a single connected component.
        std::vector<ViewId> view_ids;
        view_ids.push_back(0);
        for (size_t i = 1; i < positions_.size(); i++) {
            const ViewIdPair view_id_pair(i - 1, i);
            view_pairs_[view_id_pair] =
                CreateTwoViewInfo(view_id_pair, pose_noise);
            view_ids.push_back(i);
        }

        std::random_device rd;
        std::mt19937_64 rngg{rd()};
        while (view_pairs_.size() < num_view_pairs) {
            std::shuffle(view_ids.begin(), view_ids.end(), rngg);
            const ViewIdPair view_id_pair =
                (view_ids[0] < view_ids[1])
                    ? ViewIdPair(view_ids[0], view_ids[1])
                    : ViewIdPair(view_ids[1], view_ids[0]);
            if (view_pairs_.contains(view_id_pair)) {
                continue;
            }

            view_pairs_[view_id_pair] =
                CreateTwoViewInfo(view_id_pair, pose_noise);
        }
    }

    ViewPairInfo CreateTwoViewInfo(const ViewIdPair& view_id_pair,
                                   const double pose_noise)
    {
        CHECK_LT(view_id_pair.first, view_id_pair.second);
        ViewPairInfo info;
        info.focalLength1 = 800.0;
        info.focalLength2 = 800.0;

        // These objects will add noise to the relative pose.
        const Vector2d noise = pose_noise * Vector2d::Random();

        // Determine the relative rotation and add noise.
        info.rotation = RelativeRotationFromTwoRotations(
            con::FindOrDie(orientations_, view_id_pair.first),
            con::FindOrDie(orientations_, view_id_pair.second), noise(0));

        // Determine the relative position and add noise.
        info.position = RelativeTranslationFromTwoPositions(
            con::FindOrDie(positions_, view_id_pair.first),
            con::FindOrDie(positions_, view_id_pair.second),
            con::FindOrDie(orientations_, view_id_pair.first), noise(1));

        return info;
    }

    LinearPositionEstimator::Options options_;
    std::unordered_map<ViewId, Vector3d> positions_;
    std::unordered_map<ViewId, Vector3d> orientations_;
    std::unordered_map<ViewIdPair, ViewPairInfo> view_pairs_;
    Scene reconstruction_;
};

// FIXME: The result errors are around 1e-3, which is larger than 1e-4
TEST_F(EstimatePositionsLinearTest, SmallTestNoNoise)
{
    static const double kTolerance = 1e-4;
    static const int kNumViews = 4;
    static const int kNumTracksPerView = 50;
    static const int kNumViewPairs = 6;
    TestLinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                0.0, kTolerance);
}

TEST_F(EstimatePositionsLinearTest, SmallTestWithNoise)
{
    static const double kTolerance = 0.25;
    static const int kNumViews = 4;
    static const int kNumTracksPerView = 50;
    static const int kNumViewPairs = 6;
    static const double kPoseNoiseDegrees = 1.0;
    TestLinearPositionEstimator(kNumViews, kNumTracksPerView, kNumViewPairs,
                                kPoseNoiseDegrees, kTolerance);
}

void RelativeRotationFromTwoRotations(const Vector3d& rotation1,
                                      const Vector3d& rotation2,
                                      Vector3d* relative_rotation)
{
    Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
    ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
    ceres::AngleAxisToRotationMatrix(rotation2.data(), rotation_matrix2.data());

    const Eigen::Matrix3d relative_rotation_mat =
        rotation_matrix2 * rotation_matrix1.transpose();
    ceres::RotationMatrixToAngleAxis(relative_rotation_mat.data(),
                                     relative_rotation->data());
}

void RelativeTranslationFromTwoPositions(const Vector3d& position1,
                                         const Vector3d& position2,
                                         const Vector3d& rotation1,
                                         Vector3d* translation)
{
    Eigen::Matrix3d rotation_matrix1;
    ceres::AngleAxisToRotationMatrix(rotation1.data(), rotation_matrix1.data());
    *translation = rotation_matrix1 * (position2 - position1).normalized();
}

void TwoViewInfoFromCameras(const Camera& camera1, const Camera& camera2,
                            ViewPairInfo* info)
{
    RelativeRotationFromTwoRotations(camera1.orientationAsAngleAxis(),
                                     camera2.orientationAsAngleAxis(),
                                     &info->rotation);
    RelativeTranslationFromTwoPositions(camera1.position(), camera2.position(),
                                        camera1.orientationAsAngleAxis(),
                                        &info->position);
}

// Cameras are generated within a 2x2x2 box around the origin looking
// approximately toward the negative z-axis. The scene depth indicates roughly
// how many units away from the origin the 3D points will be when created. The
// points will always be in front of all cameras.
void TestTripletBaselineComputation(const double pixel_noise,
                                    const double scene_depth,
                                    const double tolerance)
{
    static const int kNumTracks = 100;
    static const double kFocalLength = 1000;

    // Set up 3 views.
    Camera camera1, camera2, camera3;
    camera1.setPosition(Vector3d(-1, 0.8, -0.2));
    camera1.setOrientationFromAngleAxis(0.2 * Vector3d::Random());
    camera2.setPosition(Vector3d(0.2, 0.3, -1.0));
    camera2.setOrientationFromAngleAxis(0.2 * Vector3d::Random());
    camera3.setPosition(Vector3d(0.8, -0.7, 0.3));
    camera3.setOrientationFromAngleAxis(0.2 * Vector3d::Random());

    // Add tracks.
    std::vector<Feature> feature1(kNumTracks), feature2(kNumTracks),
        feature3(kNumTracks);
    for (int i = 0; i < kNumTracks; i++) {
        const Eigen::Vector4d point = Vector3d::Random().homogeneous() +
                                      Eigen::Vector4d(0, 0, scene_depth, 0);

        // Add the observations (plus noise if applicable).
        camera1.projectPoint(point, feature1[i].pos);
        camera2.projectPoint(point, feature2[i].pos);
        camera3.projectPoint(point, feature3[i].pos);

        if (pixel_noise > 0) {
            feature1[i].pos.x() +=
                kRNG.randNorm(0.0, pixel_noise / kFocalLength);
            feature1[i].pos.y() +=
                kRNG.randNorm(0.0, pixel_noise / kFocalLength);
            feature2[i].pos.x() +=
                kRNG.randNorm(0.0, pixel_noise / kFocalLength);
            feature2[i].pos.y() +=
                kRNG.randNorm(0.0, pixel_noise / kFocalLength);
            feature3[i].pos.x() +=
                kRNG.randNorm(0.0, pixel_noise / kFocalLength);
            feature3[i].pos.y() +=
                kRNG.randNorm(0.0, pixel_noise / kFocalLength);
        }
    }

    // Compute the baseline ratios.
    Eigen::Vector3d baseline;
    ViewTripletInfo triplet;
    TwoViewInfoFromCameras(camera1, camera2, &triplet.info12);
    TwoViewInfoFromCameras(camera1, camera3, &triplet.info13);
    TwoViewInfoFromCameras(camera2, camera3, &triplet.info23);
    EXPECT_TRUE(ComputeTripletBaselineRatios(triplet, feature1, feature2,
                                             feature3, &baseline));

    // Measure the error.
    const double baseline_12 = (camera2.position() - camera1.position()).norm();
    const double baseline_13 = (camera3.position() - camera1.position()).norm();
    const double baseline_23 = (camera3.position() - camera2.position()).norm();
    const Eigen::Vector3d gt_baseline(1.0, baseline_13 / baseline_12,
                                      baseline_23 / baseline_12);

    EXPECT_NEAR(gt_baseline[0] / baseline[0], 1.0, tolerance);
    EXPECT_NEAR(gt_baseline[1] / baseline[1], 1.0, tolerance);
    EXPECT_NEAR(gt_baseline[2] / baseline[2], 1.0, tolerance);
}

TEST(ComputeTripletBaselineRatios, NoNoiseSmallScene)
{
    TestTripletBaselineComputation(0, 2, 1e-8);
}

TEST(ComputeTripletBaselineRatios, NoiseSmallScene)
{
    TestTripletBaselineComputation(1, 2, 0.05);
}

TEST(ComputeTripletBaselineRatios, NoNoiseLargeScene)
{
    TestTripletBaselineComputation(0, 20, 1e-8);
}

TEST(ComputeTripletBaselineRatios, NoiseLargeScene)
{
    TestTripletBaselineComputation(1, 20, 0.1);
}
