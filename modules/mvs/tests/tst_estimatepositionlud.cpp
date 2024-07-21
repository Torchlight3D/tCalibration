#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <tCore/ContainerUtils>
#include <tCore/Math>
#include <tCore/RandomGenerator>
#include <tMath/Eigen/Rotation>
#include <tMvs/Poses/EstimatePositionLUD>

#include "test_utils.h"

using namespace tl;

using Eigen::Vector2d;
using Eigen::Vector3d;

namespace {

RandomNumberGenerator kRNG(63);

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

// // Aligns positions to the ground truth positions via a similarity
// // transformation.
// void AlignPositions(const std::unordered_map<ViewId, Vector3d>& gt_positions,
//                     std::unordered_map<ViewId, Vector3d>* positions)
// {
//     // Collect all positions into a vector.
//     std::vector<Vector3d> gt_pos, pos;
//     for (const auto& gt_position : gt_positions) {
//         gt_pos.push_back(gt_position.second);
//         const Vector3d& position = FindOrDie(*positions, gt_position.first);
//         pos.push_back(position);
//     }

//     Eigen::Matrix3d rotation;
//     Vector3d translation;
//     double scale;
//     AlignPointCloudsUmeyama(pos, gt_pos, &rotation, &translation, &scale);

//     // Apply the similarity transformation.
//     for (auto& position : *positions) {
//         position.second = scale * (rotation * position.second) + translation;
//     }
// }

} // namespace

class EstimatePositionsLeastUnsquaredDeviationTest : public ::testing::Test
{
public:
    void TestLeastUnsquaredDeviationPositionEstimator(
        const int num_views, const int num_view_pairs, const double pose_noise,
        const double position_tolerance)
    {
        // Set up the scene.
        SetupScene(num_views);
        GetTwoViewInfos(num_view_pairs, pose_noise);

        // Estimate the positions.
        LeastUnsquaredDeviationPositionEstimator position_estimator(options_);
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

    void SetupScene(const int num_views)
    {
        // Create random views.
        for (int i = 0; i < num_views; i++) {
            // Create a random pose.
            orientations_[i] = 0.2 * Vector3d::Random();
            positions_[i] = 10.0 * Vector3d::Random();
        }
    }

    void GetTwoViewInfos(const int num_view_pairs, const double pose_noise)
    {
        // Create a single connected component.
        std::vector<ViewId> view_ids;
        view_ids.push_back(0);
        for (int i = 1; i < positions_.size(); i++) {
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
        const Eigen::Vector2d noise = pose_noise * Vector2d::Random();

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

    LeastUnsquaredDeviationPositionEstimator::Options options_;
    std::unordered_map<ViewId, Vector3d> positions_;
    std::unordered_map<ViewId, Vector3d> orientations_;
    std::unordered_map<ViewIdPair, ViewPairInfo> view_pairs_;
};

TEST_F(EstimatePositionsLeastUnsquaredDeviationTest, SmallTestNoNoise)
{
    static const double kTolerance = 1e-2;
    static const int kNumViews = 4;
    static const int kNumViewPairs = 6;
    TestLeastUnsquaredDeviationPositionEstimator(kNumViews, kNumViewPairs, 0.0,
                                                 kTolerance);
}

TEST_F(EstimatePositionsLeastUnsquaredDeviationTest, SmallTestWithNoise)
{
    static const double kTolerance = 0.1;
    static const int kNumViews = 4;
    static const int kNumViewPairs = 6;
    static const double kPoseNoiseDegrees = 1.0;
    TestLeastUnsquaredDeviationPositionEstimator(kNumViews, kNumViewPairs,
                                                 kPoseNoiseDegrees, kTolerance);
}

TEST_F(EstimatePositionsLeastUnsquaredDeviationTest, TestNoNoise)
{
    static const double kTolerance = 0.5;
    static const int kNumViews = 200;
    static const int kNumViewPairs = 500;
    static const double kPoseNoiseDegrees = 0.0;
    TestLeastUnsquaredDeviationPositionEstimator(kNumViews, kNumViewPairs,
                                                 kPoseNoiseDegrees, kTolerance);
}

TEST_F(EstimatePositionsLeastUnsquaredDeviationTest, TestWithNoise)
{
    static const double kTolerance = 1.0;
    static const int kNumViews = 200;
    static const int kNumViewPairs = 500;
    static const double kPoseNoiseDegrees = 1.0;
    TestLeastUnsquaredDeviationPositionEstimator(kNumViews, kNumViewPairs,
                                                 kPoseNoiseDegrees, kTolerance);
}
