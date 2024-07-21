#include "basics.h"

#include <algorithm>
#include <numbers>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <glog/logging.h>

#include <tMath/Polynomial>
#include <tMath/Eigen/Utils>

#include "../desc/feature.h"
#include "triangulation.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using DiagonalMatrix3d = Eigen::DiagonalMatrix<double, 3>;

std::vector<Eigen::Vector2d> NormalizeImagePoints(
    const std::vector<Eigen::Vector2d>& imgPoints,
    Matrix3d* normalization_matrix)
{
    Eigen::Map<const Matrix2Xd> imgPointsMat(imgPoints[0].data(), 2,
                                             imgPoints.size());

    // Compute centroid.
    const Vector2d centroid = imgPointsMat.rowwise().mean();

    // Calculate average RMS distance to centroid.
    const double rmsDist = std::sqrt(
        (imgPointsMat.colwise() - centroid).squaredNorm() / imgPoints.size());

    // Create normalization matrix.
    const double norm_factor = std::numbers::sqrt2 / rmsDist;
    // clang-format off
    *normalization_matrix << norm_factor, 0., -norm_factor * centroid.x(),
                             0., norm_factor, -norm_factor * centroid.y(),
                             0.,          0.,                          1.;
    // clang-format on

    // Normalize image points.
    const Matrix3Xd normalized_homog_points =
        (*normalization_matrix) * imgPointsMat.colwise().homogeneous();

    // Allocate the output vector and map an Eigen object to the underlying data
    // for efficient calculations.
    std::vector<Vector2d> imgPointsNorm;
    imgPointsNorm.resize(imgPoints.size());
    Eigen::Map<Matrix2Xd> normImgPointsMat(imgPointsNorm[0].data(), 2,
                                           imgPoints.size());
    normImgPointsMat = normalized_homog_points.colwise().hnormalized();

    return imgPointsNorm;
}

double scaleEpipolarThreshold(double threshold, int imageWidth, int imageHeight)
{
    if (imageWidth == 0 && imageHeight == 0) {
        return threshold;
    }

    constexpr auto kDefaultImageDimension = 1024.;

    const auto maxImageDimension = std::max(imageWidth, imageHeight);
    return threshold * maxImageDimension / kDefaultImageDimension;
}

double SquaredSampsonDistance(const Eigen::Matrix3d& F,
                              const Eigen::Vector2d& x,
                              const Eigen::Vector2d& y)
{
    const auto epiline_x = F * x.homogeneous();
    const auto y_1 = y.homogeneous();
    const auto numerator_sqrt = y_1.dot(epiline_x);
    const Vector4d deno{y_1.dot(F.col(0)), y_1.dot(F.col(1)), epiline_x[0],
                        epiline_x[1]};

    return numerator_sqrt * numerator_sqrt / deno.squaredNorm();
}

double squaredEpipolarDistance(const Eigen::Matrix3d& F,
                               const Eigen::Vector2d& x,
                               const Eigen::Vector2d& y)
{
    const auto epiline_x = F * x.homogeneous();
    const auto y_1 = y.homogeneous();
    const auto numerator_sqrt = y_1.dot(epiline_x);
    const auto deno = epiline_x.head<2>();

    return numerator_sqrt * numerator_sqrt / deno.squaredNorm();
}

void DecomposeEssentialMatrix(const Eigen::Matrix3d& E,
                              Eigen::Matrix3d* rotation1,
                              Eigen::Matrix3d* rotation2,
                              Eigen::Vector3d* translation)
{
    Matrix3d d;
    // clang-format off
    d << 0., 1., 0.,
        -1., 0., 0.,
         0., 0., 1.;
    // clang-format on

    const Eigen::JacobiSVD<Matrix3d> svd{
        E, Eigen::ComputeFullU | Eigen::ComputeFullV};
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    if (U.determinant() < 0) {
        U.col(2) *= -1.;
    }

    if (V.determinant() < 0) {
        V.col(2) *= -1.;
    }

    // Possible configurations.
    *rotation1 = U * d * V.transpose();
    *rotation2 = U * d.transpose() * V.transpose();
    *translation = U.col(2).normalized();
}

// x = (R2 * X + t2)
// x = (R2 * R1^t * X + t2)
// x = (R2 * (R1^t * X - t1) + t2)
// x = R2 * R1^t * X - R2 * t1 + t2
void EssentialMatrixFromTwoProjectionMatrices(const Matrix34d& pose1,
                                              const Matrix34d& pose2,
                                              Eigen::Matrix3d* E)
{
    // Create the Ematrix from the poses.
    const Matrix3d R1 = pose1.leftCols<3>();
    const Matrix3d R2 = pose2.leftCols<3>();
    const Vector3d t1 = pose1.rightCols<1>();
    const Vector3d t2 = pose2.rightCols<1>();

    // Pos1 = -R1^t * t1.
    // Pos2 = -R2^t * t2.
    // t = R1 * (pos2 - pos1).
    // t = R1 * (-R2^t * t2 + R1^t * t1)
    // t = t1 - R1 * R2^t * t2;

    // Relative transformation between to cameras.
    const Matrix3d relative_rotation = R1 * R2.transpose();
    const Vector3d translation = (t1 - relative_rotation * t2).normalized();
    *E = math::Skew(translation) * relative_rotation;
}

int GetBestPoseFromEssentialMatrix(const Eigen::Matrix3d& E,
                                   const std::vector<Feature2D2D>& corrs,
                                   Eigen::Matrix3d* _R, Eigen::Vector3d* _t)
{
    // Decompose ematrix.
    Matrix3d R1, R2;
    Vector3d t;
    DecomposeEssentialMatrix(E, &R1, &R2, &t);

    constexpr auto kNumCandiates{4};
    const std::vector rotations{R1, R1, R2, R2};
    const std::vector<Vector3d> positions{
        -R1.transpose() * t, R1.transpose() * t, -R2.transpose() * t,
        R2.transpose() * t};

    // From the 4 candidate poses, find the one with the most triangulated
    // points in front of the camera.
    std::vector<int> pointCountInFrontOfCamera(kNumCandiates, 0);
    for (auto i{0}; i < kNumCandiates; i++) {
        for (const auto& corr : corrs) {
            if (IsTriangulatedPointInFrontOfCameras(corr, rotations[i],
                                                    positions[i])) {
                ++pointCountInFrontOfCamera[i];
            }
        }
    }

    // Find the pose with the most points in front of the camera.
    const auto max = std::max_element(pointCountInFrontOfCamera.begin(),
                                      pointCountInFrontOfCamera.end());
    const auto maxIndex = std::distance(pointCountInFrontOfCamera.begin(), max);

    // Set the pose.
    *_R = rotations[maxIndex];
    *_t = positions[maxIndex];
    return *max;
}

// Given a fundmental matrix, decompose the fundmental matrix and recover focal
// lengths f1, f2 >0 such that diag([f2 f2 1]) F diag[f1 f1 1]) is a valid
// essential matrix. This assumes a principal point of (0, 0) for both cameras.
// Returns true on success, false otherwise.
bool FocalLengthsFromFundamentalMatrix(const double fmatrix[3 * 3], double* f1,
                                       double* f2)
{
    Eigen::Map<const Matrix3d> F(fmatrix);

    // Compute the epipoles for each image.
    const Vector3d epipole1 =
        F.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    const Vector3d epipole2 =
        F.transpose().jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    if (epipole1.x() == 0 || epipole2.x() == 0) {
        VLOG(3) << "Cannot recover the focal length: "
                   "Optical axes are collinear. ";
        return false;
    }

    // Find the rotation that takes epipole1 to (e_0, 0, e_2) and the
    // epipole2 to (e'_0, 0, e'_2). If we form a rotation matrix:
    // R = [ cos x  -sin x  0 ]
    //     [ sin x   cos x  0 ]
    //     [ 0       0      1 ]
    // then we can solve for the angle x such that R * e1 = (e_1, 0, e_3).
    // We can solve this simply be investigating the second row and noting that
    // e1(0) * sin x + e2 * cos x = 0.

    auto toR = [](double theta) {
        Matrix3d R;
        // clang-format off
        R << std::cos(theta), -std::sin(theta), 0.,
             std::sin(theta),  std::cos(theta), 0.,
                          0.,              0. , 1.;
        // clang-format on
        return R;
    };

    const auto theta1 = std::atan2(-epipole1(1), epipole1(0));
    const auto theta2 = std::atan2(-epipole2(1), epipole2(0));

    const auto R1 = toR(theta1);
    const auto R2 = toR(theta2);

    const auto R2_F_R1t = R2 * F * R1.transpose();

    // With the normalized epipoles, the fundamental matrix is now of the form:
    // F = [ e'_2   0    0   ] [ a b a ] [ e_2   0     0  ]
    //     [ 0      1    0   ] [ c d c ] [ 0     1     0  ]
    //     [ 0      0  -e'_1 ] [ a b a ] [ 0     0   -e_1 ]
    const Vector3d rotated_epipole1 = R1 * epipole1;
    const Vector3d rotated_epipole2 = R2 * epipole2;

    Matrix3d factorized_matrix =
        DiagonalMatrix3d(rotated_epipole2(2), 1, -rotated_epipole2(0))
            .inverse() *
        R2_F_R1t *
        DiagonalMatrix3d(rotated_epipole1(2), 1, -rotated_epipole1(0))
            .inverse();

    // For convenience, as defined above.
    const double a = factorized_matrix(0, 0);
    const double b = factorized_matrix(0, 1);
    const double c = factorized_matrix(1, 0);
    const double d = factorized_matrix(1, 1);

    const double f1f1 =
        (-a * c * rotated_epipole1(0) * rotated_epipole1(0)) /
        (a * c * rotated_epipole1(2) * rotated_epipole1(2) + b * d);
    const double f2f2 =
        (-a * b * rotated_epipole2(0) * rotated_epipole2(0)) /
        (a * b * rotated_epipole2(2) * rotated_epipole2(2) + c * d);

    if (f1f1 < 0 || f2f2 < 0) {
        VLOG(3) << "Real focal length values could not be extracted.";
        return false;
    }

    *f1 = sqrt(f1f1);
    *f2 = sqrt(f2f2);
    return true;
}

bool SharedFocalLengthsFromFundamentalMatrix(const double fmatrix[3 * 3],
                                             double* f)
{
    // Construct a "semi-calibrated" matrix G with all intrinsics but focal
    // length accounted for. In our case, the fundamental matrix is equivalent
    // to G because we assume the fundmental matrix was computed with all the
    // known intrinsics effects removed.
    Eigen::Map<const Matrix3d> G(fmatrix);

    // Compute the SVD of the semi-calibrated fundamental matrix.
    Eigen::JacobiSVD<Matrix3d> svd{G,
                                   Eigen::ComputeFullU | Eigen::ComputeFullV};
    const auto& U = svd.matrixU();
    const auto& V = svd.matrixV();
    const double a = svd.singularValues()(0);
    const double b = svd.singularValues()(1);

    // Construct the quadratic equation that reveals the focal length.
    const double U20 = U(2, 0);
    const double U21 = U(2, 1);
    const double V20 = V(2, 0);
    const double V21 = V(2, 1);
    const double U20_sq = U20 * U20;
    const double U21_sq = U21 * U21;
    const double V20_sq = V20 * V20;
    const double V21_sq = V21 * V21;
    VectorXd coeffs(3);
    coeffs(0) = a * a * (1. - U20_sq) * (1. - V20_sq) -
                b * b * (1. - U21_sq) * (1. - V21_sq);
    coeffs(1) = a * a * (U20_sq + V20_sq - 2. * U20_sq * V20_sq) -
                b * b * (U21_sq + V21_sq - 2. * U21_sq * V21_sq);
    coeffs(2) = a * a * U20_sq * V20_sq - b * b * U21_sq * V21_sq;

    // Solve the quadratic equation. The roots provide the square of the focal
    // length.
    VectorXd real_roots, imaginary_roots;
    if (!ceres::internal::FindPolynomialRoots(coeffs, &real_roots,
                                              &imaginary_roots)) {
        return false;
    }

    // If niether root is positive then no valid solution exists. If one of the
    // roots is negative then it leads to an imaginary value for the focal
    // length so we can immediately return the other value as the solution.
    if (real_roots(0) < 0 && real_roots(1) < 0) {
        return false;
    }
    if (real_roots(0) < 0) {
        *f = std::sqrt(real_roots(1));
    }
    else if (real_roots(1) < 0) {
        *f = std::sqrt(real_roots(0));
    }
    else {
        // If we reach this point then the roots are both positive and so we
        // disambiguate the roots by selecting the root that best satisfies the
        // linear constraint.
        const double c1 =
            a * U20 * U21 * (1.0 - V20_sq) + b * V20 * V21 * (1.0 - U21_sq);
        const double c2 = U21 * V20 * (a * U20 * V20 + b * U21 * V21);
        const double root1_val = c1 * real_roots(0) + c2;
        const double root2_val = c1 * real_roots(1) + c2;
        if (std::abs(root1_val) < std::abs(root2_val)) {
            *f = std::sqrt(real_roots(0));
        }
        else {
            *f = std::sqrt(real_roots(1));
        }
    }

    return true;
}

void ProjectionMatricesFromFundamentalMatrix(const double fmatrix[3 * 3],
                                             double pmatrix1[3 * 4],
                                             double pmatrix2[3 * 4])
{
    Eigen::Map<const Matrix3d> F{fmatrix};
    const Vector3d right_epipole =
        F.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();

    Eigen::Map<Matrix3d>(pmatrix1, 3, 3) = Matrix3d::Identity();
    Eigen::Map<Vector3d>(pmatrix1 + 9, 3) = Vector3d::Zero();

    Eigen::Map<Matrix3d>(pmatrix2, 3, 3) =
        math::Skew(right_epipole) * F.transpose();
    Eigen::Map<Vector3d>(pmatrix2 + 9, 3) = right_epipole;
}

void FundamentalMatrixFromProjectionMatrices(const double pmatrix1[3 * 4],
                                             const double pmatrix2[3 * 4],
                                             double fmatrix[3 * 3])
{
    Eigen::Map<const Matrix34d> P1{pmatrix1};
    Eigen::Map<const Matrix34d> P2{pmatrix2};
    Eigen::Map<Matrix3d> F{fmatrix};

    constexpr int kIndex1[3]{1, 2, 0};
    constexpr int kIndex2[3]{2, 0, 1};

    Matrix4d temp_mat;
    for (int r = 0; r < 3; r++) {
        temp_mat.row(2) = P1.row(kIndex1[r]);
        temp_mat.row(3) = P1.row(kIndex2[r]);
        for (int c = 0; c < 3; c++) {
            temp_mat.row(0) = P2.row(kIndex1[c]);
            temp_mat.row(1) = P2.row(kIndex2[c]);
            F(r, c) = temp_mat.determinant();
        }
    }
}

void EssentialMatrixFromFundamentalMatrix(const double fmatrix[3 * 3],
                                          double f1, double f2,
                                          double ematrix[3 * 3])
{
    Eigen::Map<Matrix3d>{ematrix} = DiagonalMatrix3d{f2, f2, 1.} *
                                    Eigen::Map<const Matrix3d>{fmatrix} *
                                    DiagonalMatrix3d{f1, f1, 1.};
}

void ComposeFundamentalMatrix(double f1, double f2,
                              const double rotation[3 * 3],
                              const double translation[3],
                              double fmatrix[3 * 3])
{
    const Matrix3d K1_inv = DiagonalMatrix3d{f1, f1, 1.}.inverse();
    const Matrix3d K2_inv = DiagonalMatrix3d{f2, f2, 1.}.inverse();

    const Eigen::Map<const Matrix3d> R{rotation};
    const Eigen::Map<const Vector3d> t{translation};

    Eigen::Map<Matrix3d>{fmatrix} = K2_inv * math::Skew(t.eval()) * R * K1_inv;
}

} // namespace tl
