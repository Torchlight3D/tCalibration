﻿#include "p3p.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <glog/logging.h>

#include <tMath/Polynomial>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace {

// Solves for cos(theta) that will describe the rotation of the plane from
// intermediate world frame to intermediate camera frame. The method returns the
// roots of a quartic (i.e. solutions to cos(alpha) ) and several factors that
// are needed for back-substitution.
int SolvePlaneRotation(
    const std::array<Eigen::Vector3d, 3>& normalized_image_points,
    const Eigen::Vector3d& intermediate_image_point,
    const Eigen::Vector3d& intermediate_world_point, double d_12,
    double cos_theta[4], double cot_alphas[4], double* b)
{
    // Calculate these parameters ahead of time for reuse and
    // readability. Notation for these variables is consistent with the notation
    // from the paper.
    const double f_1 =
        intermediate_image_point[0] / intermediate_image_point[2];
    const double f_2 =
        intermediate_image_point[1] / intermediate_image_point[2];
    const double p_1 = intermediate_world_point[0];
    const double p_2 = intermediate_world_point[1];
    const double cos_beta =
        normalized_image_points[0].dot(normalized_image_points[1]);
    *b = 1.0 / (1.0 - cos_beta * cos_beta) - 1.0;

    if (cos_beta < 0) {
        *b = -std::sqrt(*b);
    }
    else {
        *b = std::sqrt(*b);
    }

    // Definition of temporary variables for readability in the coefficients
    // calculation.
    const double f_1_pw2 = f_1 * f_1;
    const double f_2_pw2 = f_2 * f_2;
    const double p_1_pw2 = p_1 * p_1;
    const double p_1_pw3 = p_1_pw2 * p_1;
    const double p_1_pw4 = p_1_pw3 * p_1;
    const double p_2_pw2 = p_2 * p_2;
    const double p_2_pw3 = p_2_pw2 * p_2;
    const double p_2_pw4 = p_2_pw3 * p_2;
    const double d_12_pw2 = d_12 * d_12;
    const double b_pw2 = (*b) * (*b);

    // Computation of coefficients of 4th degree polynomial.
    VectorXd coefficients(5);
    coefficients(0) = -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;
    coefficients(1) = 2.0 * p_2_pw3 * d_12 * (*b) +
                      2.0 * f_2_pw2 * p_2_pw3 * d_12 * (*b) -
                      2.0 * f_2 * p_2_pw3 * f_1 * d_12;
    coefficients(2) =
        -f_2_pw2 * p_2_pw2 * p_1_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2 -
        f_2_pw2 * p_2_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw4 + p_2_pw4 * f_1_pw2 +
        2.0 * p_1 * p_2_pw2 * d_12 +
        2.0 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * (*b) -
        p_2_pw2 * p_1_pw2 * f_1_pw2 + 2.0 * p_1 * p_2_pw2 * f_2_pw2 * d_12 -
        p_2_pw2 * d_12_pw2 * b_pw2 - 2.0 * p_1_pw2 * p_2_pw2;
    coefficients(3) = 2.0 * p_1_pw2 * p_2 * d_12 * (*b) +
                      2.0 * f_2 * p_2_pw3 * f_1 * d_12 -
                      2.0 * f_2_pw2 * p_2_pw3 * d_12 * (*b) -
                      2.0 * p_1 * p_2 * d_12_pw2 * (*b);
    coefficients(4) = -2 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * (*b) +
                      f_2_pw2 * p_2_pw2 * d_12_pw2 + 2.0 * p_1_pw3 * d_12 -
                      p_1_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw2 * p_1_pw2 -
                      p_1_pw4 - 2.0 * f_2_pw2 * p_2_pw2 * p_1 * d_12 +
                      p_2_pw2 * f_1_pw2 * p_1_pw2 +
                      f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;

    // Computation of roots.
    VectorXd roots;
    ceres::internal::FindPolynomialRoots(coefficients, &roots, nullptr);

    // Calculate cot(alpha) needed for back-substitution.
    for (int i = 0; i < roots.size(); i++) {
        cos_theta[i] = roots(i);
        cot_alphas[i] = (-f_1 * p_1 / f_2 - cos_theta[i] * p_2 + d_12 * (*b)) /
                        (-f_1 * cos_theta[i] * p_2 / f_2 + p_1 - d_12);
    }

    return static_cast<int>(roots.size());
}

// Given the complete transformation between intermediate world and camera
// frames (parameterized by cos_theta and cot_alpha), back-substitute the
// solution and get an absolute camera pose.
void Backsubstitute(const Eigen::Matrix3d& intermediate_world_frame,
                    const Eigen::Matrix3d& intermediate_camera_frame,
                    const Eigen::Vector3d& world_point_0, double cos_theta,
                    double cot_alpha, double d_12, double b,
                    Eigen::Vector3d* translation, Eigen::Matrix3d* rotation)
{
    const double sin_theta = std::sqrt(1. - cos_theta * cos_theta);
    const double sin_alpha = std::sqrt(1. / (cot_alpha * cot_alpha + 1.0));
    double cos_alpha = std::sqrt(1.0 - sin_alpha * sin_alpha);

    if (cot_alpha < 0) {
        cos_alpha = -cos_alpha;
    }

    // Get the camera position in the intermediate world frame
    // coordinates. (Eq. 5 from the paper).
    const Vector3d c_nu(
        d_12 * cos_alpha * (sin_alpha * b + cos_alpha),
        cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha),
        sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha));

    // Transform c_nu into world coordinates. Use a Map to put the solution
    // directly into the output.
    *translation = world_point_0 + intermediate_world_frame.transpose() * c_nu;

    // Construct the transformation from the intermediate world frame to the
    // intermediate camera frame.
    Matrix3d intermediate_world_to_camera_rotation;
    intermediate_world_to_camera_rotation << -cos_alpha, -sin_alpha * cos_theta,
        -sin_alpha * sin_theta, sin_alpha, -cos_alpha * cos_theta,
        -cos_alpha * sin_theta, 0, -sin_theta, cos_theta;

    // Construct the rotation matrix.
    *rotation = (intermediate_world_frame.transpose() *
                 intermediate_world_to_camera_rotation.transpose() *
                 intermediate_camera_frame)
                    .transpose();

    // Adjust translation to account for rotation.
    *translation = -(*rotation) * (*translation);
}

} // namespace

bool P3P(const std::vector<Eigen::Vector2d>& imagePoints,
         const std::vector<Eigen::Vector3d>& objectPoints,
         std::vector<Eigen::Matrix3d>& rotations,
         std::vector<Eigen::Vector3d>& translations)
{
    constexpr int kPatchSize{3};

    CHECK_EQ(imagePoints.size(), objectPoints.size());
    CHECK_GE(imagePoints.size(), kPatchSize);

    std::array<Vector3d, kPatchSize> imgPoints;
    std::array<Vector3d, kPatchSize> objPoints;
    for (int i{0}; i < kPatchSize; ++i) {
        imgPoints[i] = imagePoints[i].homogeneous().normalized();
        objPoints[i] = objectPoints[i];
    }

    // Check if the points are collinear, or there are no possible solutions.
    constexpr double kTolerance{1e-6};
    Vector3d vec10 = objPoints[1] - objPoints[0];
    Vector3d vec20 = objPoints[2] - objPoints[0];
    if (vec10.cross(vec20).squaredNorm() < kTolerance) {
        VLOG(2) << "The 3 world points are collinear! "
                   "No solution for absolute pose exits.";
        return false;
    }

    // Create intermediate camera frame such that the x axis is in the direction
    // of one of the normalized image points, and the origin is the same as the
    // absolute camera frame. This is a rotation defined as the transformation:
    //                     T = [tx, ty, tz]
    // where
    // tx = f0,
    // tz = (f0 x f1) / ||f0 x f1||
    // ty = tx x tz,
    // and f0, f1, f2 are the normalized image points.
    Matrix3d T;
    T.row(0) = imgPoints[0];
    T.row(2) = imgPoints[0].cross(imgPoints[1]).normalized();
    T.row(1) = T.row(2).cross(T.row(0));

    // Project the third world point into the intermediate camera frame.
    Vector3d intermediate_image_point = T * imgPoints[2];

    // Enforce that the intermediate_image_point is in front of the intermediate
    // camera frame. If the point is behind the camera frame, recalculate the
    // intermediate camera frame by swapping which feature we align the x axis
    // to.
    if (intermediate_image_point[2] > 0.) {
        std::swap(imgPoints[0], imgPoints[1]);

        T.row(0) = imgPoints[0];
        T.row(2) = imgPoints[0].cross(imgPoints[1]).normalized();
        T.row(1) = T.row(2).cross(T.row(0));

        intermediate_image_point = T * imgPoints[2];

        std::swap(objPoints[0], objPoints[1]);
        vec10 = objPoints[1] - objPoints[0];
        vec20 = objPoints[2] - objPoints[0];
    }

    // Create the intermediate world frame transformation that has the
    // origin at world_points[0] and the x-axis in the direction of
    // world_points[1]. This is defined by the transformation:
    //                    N = [nx, ny, nz]
    // where
    // nx = (p1 - p0) / ||p1 - p0||
    // nz = nx x (p2 - p0) / || nx x (p2 -p0) ||
    // ny = nz x nx
    // and p0, p1, p2 are the world points.
    Matrix3d N;
    N.row(0) = vec10.normalized();
    N.row(2) = N.row(0).cross(vec20).normalized();
    N.row(1) = N.row(2).cross(N.row(0));

    // Transform world_point[2] to the intermediate world frame coordinates.
    Vector3d intermediate_world_point = N * vec20;

    // Distance from world_points[1] to the intermediate world frame origin.
    const double d_12 = vec10.norm();

    // Solve for the cos(theta) that will give us the transformation from
    // intermediate world frame to intermediate camera frame. We also get the
    // cot(alpha) for each solution necessary for back-substitution.
    double cos_theta[4];
    double cot_alphas[4];
    double b;
    const int num_solutions = SolvePlaneRotation(
        imgPoints, intermediate_image_point, intermediate_world_point, d_12,
        cos_theta, cot_alphas, &b);

    // Backsubstitution of each solution
    translations.resize(num_solutions);
    rotations.resize(num_solutions);
    for (int i = 0; i < num_solutions; i++) {
        Backsubstitute(N, T, objPoints[0], cos_theta[i], cot_alphas[i], d_12, b,
                       &translations[i], &rotations[i]);
    }

    return num_solutions > 0;
}

} // namespace tl
