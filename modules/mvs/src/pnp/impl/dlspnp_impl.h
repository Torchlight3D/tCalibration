#pragma once

#include <Eigen/Core>

namespace tl {

// Put these methods in a nested namespace so that they are not part of the
// common public theia namespace.
namespace internal {

using Matrix39d = Eigen::Matrix<double, 3, 9>;
using Matrix9d = Eigen::Matrix<double, 9, 9>;

// Transforms a 3-vector in a 3x9 matrix such that:
// R * v = LeftMultiplyMatrix(v) * vec(R)
// Where R is a rotation matrix and vec(R) converts R to a 9x1 vector.
Matrix39d LeftMultiplyMatrix(const Eigen::Vector3d& v);

// Extracts the coefficients of the Jacobians of the LS cost function (which is
// parameterized by the 3 rotation coefficients s1, s2, s3).
void ExtractJacobianCoefficients(const Matrix9d& ls_cost_coefficients,
                                 double f1_coeff[20], double f2_coeff[20],
                                 double f3_coeff[20]);

// Constructs a Macaulay matrix to solve the system of equations using the
// polynomial coefficients from the jacobians.
Eigen::MatrixXd CreateMacaulayMatrix(const double f1_coeff[20],
                                     const double f2_coeff[20],
                                     const double f3_coeff[20],
                                     const double rand_term[4]);

} // namespace internal
} // namespace tl
