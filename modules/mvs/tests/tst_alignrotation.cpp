#include <Eigen/Geometry>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <tCore/Math>
#include <tMvs/Poses/AlignRotations>

using namespace tl;

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

class AlignRotationsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        //
        _opts.noise.reset();
    }

    void TearDown() override {}

    void Execute()
    {
        std::vector<Vector3d> gt_rotations(_opts.numViews);
        std::vector<Vector3d> rotations(_opts.numViews);

        Matrix3d rotation_transformation =
            AngleAxisd{15., Vector3d::Random().normalized()}.toRotationMatrix();
        for (int i{0}; i < _opts.numViews; ++i) {
            gt_rotations[i] = Vector3d::Random();
            rotations[i] = gt_rotations[i];
            applyRotation(rotation_transformation, _opts.noise, &rotations[i]);
        }

        AlignRotations(gt_rotations, &rotations);

        for (auto i{0}; i < _opts.numViews; ++i) {
            EXPECT_LT((gt_rotations[i] - rotations[i]).norm(),
                      _ref.maxAngleDiff);
        }
    }

protected:
    struct
    {
        std::optional<double> noise = {}; // deg
        int numViews = 10;
    } _opts;

    struct
    {
        double maxAngleDiff = 1.;
    } _ref;

private:
    void applyRotation(const Eigen::Matrix3d& rotation,
                       const std::optional<double>& noise,
                       Eigen::Vector3d* orientation)
    {
        const Matrix3d extra = noise.has_value()
                                   ? AngleAxisd{math::degToRad(noise.value()),
                                                Vector3d::Random().normalized()}
                                         .toRotationMatrix()
                                   : Matrix3d::Identity();

        Matrix3d R;
        ceres::AngleAxisToRotationMatrix(
            orientation->data(), ceres::ColumnMajorAdapter3x3(R.data()));
        const Matrix3d R_trans = R * (extra * rotation);

        ceres::RotationMatrixToAngleAxis(
            ceres::ColumnMajorAdapter3x3(R_trans.data()), orientation->data());
    }
};

TEST_F(AlignRotationsTest, NoNoise)
{
    _opts.numViews = 20;
    _ref.maxAngleDiff = 1e-8;

    Execute();
}

TEST_F(AlignRotationsTest, WithNoise)
{
    _opts.numViews = 20;
    _opts.noise = 1.;
    _ref.maxAngleDiff = 5e-2;

    Execute();
}
