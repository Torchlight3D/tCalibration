#pragma once

#include <tMath/Eigen/Types>

namespace tl {

// TODO: Change yx to fy, which is more intuitive
Eigen::Matrix3d intrinsicsToCalibrationMatrix(double fx, double skew,
                                              double y_x, double cx, double cy);

void calibrationMatrixToIntrinsics(const Eigen::Matrix3d& kmatrix, double* fx,
                                   double* skew, double* y_x, double* cx,
                                   double* cy);

bool decomposeProjectionMatrix(const Matrix34d& pmatrix, Eigen::Matrix3d& K,
                               Eigen::Vector3d& rvec, Eigen::Vector3d& tvec);

bool composeProjectionMatrix(const Eigen::Matrix3d& K,
                             const Eigen::Vector3d& rvec,
                             const Eigen::Vector3d& tvec, Matrix34d& pmatrix);

// Projects a 3x3 matrix to the rotation matrix in SO3 space with the closest
// Frobenius norm. For a matrix with an SVD decomposition M = USV, the nearest
// rotation matrix is R = UV'.
Eigen::Matrix3d projectToRotationMatrix(const Eigen::Matrix3d& matrix);

} // namespace tl
