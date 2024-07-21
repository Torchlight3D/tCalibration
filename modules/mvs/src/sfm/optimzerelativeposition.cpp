#include "optimzerelativeposition.h"

#include <ceres/rotation.h>
#include <glog/logging.h>

#include "../epipolar/triangulation.h"
#include "../desc/feature.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {

// Creates the constraint matrix such that ||A * t|| is minimized, where A is
//                     R_i * f_i x R_j * f_j.
// Given known rotations, we can solve for the relative translation from this
// constraint matrix.
void CreateConstraintMatrix(const std::vector<Feature2D2D>& corrs,
                            const Eigen::Vector3d& rvec1,
                            const Eigen::Vector3d& rvec2,
                            Eigen::MatrixXd* constraint_matrix)
{
    constraint_matrix->resize(3, corrs.size());

    Matrix3d R1, R2;
    ceres::AngleAxisToRotationMatrix(rvec1.data(),
                                     ceres::ColumnMajorAdapter3x3(R1.data()));
    ceres::AngleAxisToRotationMatrix(rvec2.data(),
                                     ceres::ColumnMajorAdapter3x3(R2.data()));

    for (int i = 0; i < corrs.size(); i++) {
        // R_i' * p_i
        const Vector3d R1_p1 =
            R1.transpose() * corrs[i].feature1.pos.homogeneous();
        // R_j' * p_j
        const Vector3d R2_p2 =
            R2.transpose() * corrs[i].feature2.pos.homogeneous();

        // [(R_j' * p_j) x (R_i' * p_i)]' * R_i'
        constraint_matrix->col(i) =
            R2_p2.cross(R1_p1).transpose() * R1.transpose();
    }
}

// Determines if the majority of the points are in front of the cameras. This is
// useful for determining the sign of the relative position. Returns true if
// more than 50% of correspondences are in front of both cameras and false
// otherwise.
bool MajorityOfPointsInFrontOfCameras(const std::vector<Feature2D2D>& corrs,
                                      const Eigen::Vector3d& rvec1,
                                      const Eigen::Vector3d& rvec2,
                                      const Eigen::Vector3d& t_ji)
{
    // Compose the relative rotation.
    Matrix3d R_i, R_j;
    ceres::AngleAxisToRotationMatrix(rvec1.data(),
                                     ceres::ColumnMajorAdapter3x3(R_i.data()));
    ceres::AngleAxisToRotationMatrix(rvec2.data(),
                                     ceres::ColumnMajorAdapter3x3(R_j.data()));

    const Matrix3d R_ji = R_j * R_i.transpose();

    // Check cheirality.
    const auto count = std::count_if(
        corrs.cbegin(), corrs.cend(), [&R_ji, &t_ji](const auto& corr) {
            return IsTriangulatedPointInFrontOfCameras(corr, R_ji, t_ji);
        });

    return count * 2 > corrs.size();
}

} // namespace

bool OptimizeRelativePositionWithKnownRotation(
    const std::vector<Feature2D2D>& corrs, const Eigen::Vector3d& rotation1,
    const Eigen::Vector3d& rotation2, Eigen::Vector3d* relative_position)
{
    CHECK_NOTNULL(relative_position);

    // Set the initial relative position to random. This helps avoid a bad local
    // minima that is achieved from poor initialization.
    relative_position->setRandom();

    // Constants used for the IRLS solving.
    constexpr double eps = 1e-5;
    constexpr int kMaxIterations = 100;
    constexpr int kMaxInnerIterations = 10;
    constexpr double kMinWeight = 1e-7;

    // Create the constraint matrix from the known correspondences and
    // rotations.
    MatrixXd constraint_matrix;
    CreateConstraintMatrix(corrs, rotation1, rotation2, &constraint_matrix);

    // Initialize the weighting terms for each correspondence.
    VectorXd weights(corrs.size());
    weights.setConstant(1.);

    // Solve for the relative positions using a robust IRLS.
    double cost = 0;
    int num_inner_iterations = 0;
    for (int i = 0;
         i < kMaxIterations && num_inner_iterations < kMaxInnerIterations;
         i++) {
        // Limit the minimum weight at kMinWeight.
        weights = (weights.array() < kMinWeight).select(kMinWeight, weights);

        // Apply the weights to the constraint matrix.
        const Matrix3d lhs = constraint_matrix *
                             weights.asDiagonal().inverse() *
                             constraint_matrix.transpose();

        // Solve for the relative position which is the null vector of the
        // weighted constraints.
        const Vector3d new_relative_position =
            lhs.jacobiSvd(Eigen::ComputeFullU).matrixU().rightCols<1>();

        // Update the weights based on the current errors.
        weights = (new_relative_position.transpose() * constraint_matrix)
                      .array()
                      .abs();

        // Compute the new cost.
        const double new_cost = weights.sum();

        // Check for convergence.
        const double delta = std::max(std::abs(cost - new_cost),
                                      1. - new_relative_position.squaredNorm());

        // If we have good convergence, attempt an inner iteration.
        if (delta <= eps) {
            ++num_inner_iterations;
        }
        else {
            num_inner_iterations = 0;
        }

        cost = new_cost;
        *relative_position = new_relative_position;
    }

    // The position solver above does not consider the sign of the relative
    // position. We can determine the sign by choosing the sign that puts the
    // most points in front of the camera.
    if (!MajorityOfPointsInFrontOfCameras(corrs, rotation1, rotation2,
                                          *relative_position)) {
        *relative_position *= -1.;
    }

    return true;
}

} // namespace tl
