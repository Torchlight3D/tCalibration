#pragma once

#include <Eigen/Core>

namespace tl {

using Matrix10d = Eigen::Matrix<double, 10, 10>;
using Vector10d = Eigen::Matrix<double, 10, 1>;
using RowMajorMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// Builds an action matrix for estimating rotations given Upnp cost parameters.
// The function returns the 16x16 action matrix, which tipically it is used
// when minimal samples are passed. This function can be slow if used in a
// RANSAC loop, due to a Gauss-Jordan elimination applied to the template
// matrix.
//
// Params:
//   a_matrix:  The quadratic penalty matrix.
//   b_vector:  The linear penalty vector.
//   template_matrix:  The template matrix buffer.
Eigen::Matrix<double, 16, 16> BuildActionMatrix(
    const Matrix10d& a_matrix, const Vector10d& b_vector,
    RowMajorMatrixXd* template_matrix);

inline Eigen::Matrix<double, 16, 16> BuildActionMatrix(
    const Matrix10d& a_matrix, const Vector10d& b_vector)
{
    RowMajorMatrixXd template_matrix;
    return BuildActionMatrix(a_matrix, b_vector, &template_matrix);
}

using Matrix10d = Eigen::Matrix<double, 10, 10>;
using Vector10d = Eigen::Matrix<double, 10, 1>;
using RowMajorMatrixXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// Builds an action matrix for estimating rotations given Upnp cost parameters.
// The function returns the 8x8 action matrix since exploits symmetry in the
// underlying polynomial system of Upnp.
//
// Params:
//   a_matrix:  The quadratic penalty matrix.
//   b_vector:  The linear penalty vector.
//   template_matrix:  The template matrix buffer.
Eigen::Matrix<double, 8, 8> BuildActionMatrixUsingSymmetry(
    const Matrix10d& a_matrix, const Vector10d& b_vector,
    RowMajorMatrixXd* template_matrix);

inline Eigen::Matrix<double, 8, 8> BuildActionMatrixUsingSymmetry(
    const Matrix10d& a_matrix, const Vector10d& b_vector)
{
    RowMajorMatrixXd template_matrix;
    return BuildActionMatrixUsingSymmetry(a_matrix, b_vector, &template_matrix);
}

} // namespace tl
