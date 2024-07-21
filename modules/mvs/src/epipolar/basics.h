#pragma once

#include <vector>

#include <Eigen/Core>

#include <tMath/Eigen/Types>

namespace tl {

// Computes the normalization matrix transformation that centers image points
// around the origin with an average distance of sqrt(2) to the centroid.
// Returns the transformation matrix and the transformed points. This assumes
// that no points are at infinity.
std::vector<Eigen::Vector2d> NormalizeImagePoints(
    const std::vector<Eigen::Vector2d>& imagePoints,
    Eigen::Matrix3d* normalizationMatrix);

// Default epipolar error is based on image of size 1024x1024. For images of
// different resolutions, this function will scale the threshold based on the
// maximum image dimensions. If the image dimensions are not correctly set, the
// input value is returned.
double scaleEpipolarThreshold(double threshold, int imageWidth,
                              int imageHeight);

// Calculate epipolar distance (error)
// For an E or F that is defined such that [y 1]' * E * [x 1] = 0
// Calculate Sampson distance for two corresponding points and an essential
// or fundamental matrix (ref. MVG eq. 11.9)
double SquaredSampsonDistance(const Eigen::Matrix3d& F,
                              const Eigen::Vector2d& x,
                              const Eigen::Vector2d& y);
inline auto sampsonDistance(const Eigen::Matrix3d& F, const Eigen::Vector2d& x,
                            const Eigen::Vector2d& y)
{
    return std::sqrt(SquaredSampsonDistance(F, x, y));
}

double squaredEpipolarDistance(const Eigen::Matrix3d& F,
                               const Eigen::Vector2d& x,
                               const Eigen::Vector2d& y);
inline auto epipolarDistance(const Eigen::Matrix3d& F, const Eigen::Vector2d& x,
                             const Eigen::Vector2d& y)
{
    return std::sqrt(squaredEpipolarDistance(F, x, y));
}

// Decomposes the essential matrix into the rotation R and translation t
// such that E can be any of the four candidate solutions:
// [rotation1 |  translation], [rotation1 | -translation],
// [rotation2 |  translation], [rotation2 | -translation].
void DecomposeEssentialMatrix(const Eigen::Matrix3d& E,
                              Eigen::Matrix3d* rotation1,
                              Eigen::Matrix3d* rotation2,
                              Eigen::Vector3d* translation);

// Create an essential matrix from two projection matrices of the form [R|t].
void EssentialMatrixFromTwoProjectionMatrices(const Matrix34d& pose1,
                                              const Matrix34d& pose2,
                                              Eigen::Matrix3d* E);

struct Feature2D2D;

// Chooses the best pose of the 4 possible poses that can be computed from the
// essential matrix. The best pose is chosen as the pose that triangulates the
// most points in front of both cameras and the number of triangulated points is
// returned.
int GetBestPoseFromEssentialMatrix(
    const Eigen::Matrix3d& E,
    const std::vector<Feature2D2D>& normalized_correspondences,
    Eigen::Matrix3d* rotation, Eigen::Vector3d* position);

// Given a fundmental matrix, decompose the fundmental matrix and recover focal
// lengths f1, f2 >0. This assumes a principal point of (0, 0) for both cameras.
//
// Returns true on success, false otherwise.
bool FocalLengthsFromFundamentalMatrix(const double fmatrix[3 * 3], double* f1,
                                       double* f2);

// Given a fundamental matrix that relates two cameras with the same intrinsics,
// extract the shared focal length. This assumes that the fundamental matrix was
// computed with the effect of all non-focal length intrinsics (e.g., principal
// point, aspect ratio, etc.) removed.
// Given a fundamental matrix that relates two cameras with the same intrinsics,
// extract the shared focal length. The method is provided in the article:
//   "On Focal Length Calibration from Two Views" by Peter Sturm (CVPR 2001).
bool SharedFocalLengthsFromFundamentalMatrix(const double fmatrix[3 * 3],
                                             double* f);

// Computes the projection matrices corresponding to the fundamental matrix such
// that if y^t * F * x = 0, pmatrix1 corresponds to the camera observing y and
// pmatrix2 corresponds to the camera observing x.
void ProjectionMatricesFromFundamentalMatrix(const double fmatrix[3 * 3],
                                             double pmatrix1[3 * 4],
                                             double pmatrix2[3 * 4]);

// Constructs projection matrices from the input fundamental matrix. The
// fundamental matrix is such that point1^t * fmatrix * point2 = 0 for point1 in
// the image corresponding to pmatrix1 and point2 in the image corresponding to
// pmatrix2.

// Ported from Hartley and Zisserman:
// http://www.robots.ox.ac.uk/~vgg/hzbook/code/vgg_multiview/vgg_F_from_P.m
void FundamentalMatrixFromProjectionMatrices(const double pmatrix1[3 * 4],
                                             const double pmatrix2[3 * 4],
                                             double fmatrix[3 * 3]);

// Extracts the essential matrix such that diag([f2 f2 1]) F diag[f1 f1 1]) is a
// valid essential matrix.
void EssentialMatrixFromFundamentalMatrix(const double fmatrix[3 * 3],
                                          double f1, double f2,
                                          double ematrix[3 * 3]);

// Composes a fundamental matrix such that:
//    F = K_2^-1 * [t]_x * R * K_2^-1
// where K_i is the calibration matrix for image i. The fundamental matrix F is
// thus the matrix that transfers points from image 1 to lines in image 2.
void ComposeFundamentalMatrix(double focalLength1, double focalLength2,
                              const double rotation[3 * 3],
                              const double translation[3],
                              double fmatrix[3 * 3]);

} // namespace tl
