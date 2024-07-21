#pragma once

#include <unordered_map>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

#include "rotationestimator.h"

namespace tl {

// Brief:
// The error in two global rotations based on the current estimates for the
// global rotations and the relative rotation such that R{i, j} = R_j * R_i'.
//
// NOTE: Make this public to enable unit test
struct PairwiseRotationError
{
    const Eigen::Vector3d relative_rotation_;
    const double weight_;

    PairwiseRotationError(const Eigen::Vector3d& relative_rotation,
                          double weight)
        : relative_rotation_(relative_rotation), weight_(weight)
    {
    }

    // The error is given by the rotation loop error as specified above. We
    // return 3 residuals to give more opportunity for optimization.
    template <typename T>
    bool operator()(const T* rvec1, const T* rvec2, T* residuals) const
    {
        using Eigen::Matrix3d;
        using Mat3 = Eigen::Matrix3<T>;
        using Vec3 = Eigen::Vector3<T>;

        // Convert angle axis rotations to rotation matrices.
        Mat3 R1, R2;
        ceres::AngleAxisToRotationMatrix(
            rvec1, ceres::ColumnMajorAdapter3x3(R1.data()));
        ceres::AngleAxisToRotationMatrix(
            rvec2, ceres::ColumnMajorAdapter3x3(R2.data()));
        Matrix3d R;
        ceres::AngleAxisToRotationMatrix(
            relative_rotation_.data(), ceres::ColumnMajorAdapter3x3(R.data()));

        // Compute the loop rotation from the two global rotations.
        const Mat3 loop_rotation_mat = R2 * R1.transpose();
        // Compute the error matrix between the expected relative rotation and
        // the observed relative rotation
        const Mat3 error_rotation_mat =
            loop_rotation_mat * R.cast<T>().transpose();
        Vec3 error_rotation;
        ceres::RotationMatrixToAngleAxis(
            ceres::ColumnMajorAdapter3x3(error_rotation_mat.data()),
            error_rotation.data());

        Eigen::Map<Vec3>{residuals} = weight_ * error_rotation;

        return true;
    }

    static ceres::CostFunction* create(const Eigen::Vector3d& relative_rotation,
                                       double weight);
};

// Brief:
// Computes the global rotations given relative rotations and an initial guess
// for the global orientations. Nonlinear optimization is performed with Ceres
// using a SoftL1 loss function to be robust to outliers.
class NonlinearRotationEstimator final : public RotationEstimator
{
public:
    explicit NonlinearRotationEstimator(double robust_loss_width = 0.1);

    bool EstimateRotations(
        const std::unordered_map<ViewIdPair, ViewPairInfo>& viewPairs,
        std::unordered_map<ViewId, Eigen::Vector3d>* orientations) override;

private:
    const double robust_loss_width_;
};

} // namespace tl
