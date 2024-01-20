#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tl {

using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Matrix39 = Eigen::Matrix<double, 3, 9>;
using Matrix93 = Eigen::Matrix<double, 9, 3>;
using Matrix96 = Eigen::Matrix<double, 9, 6>;
using Matrix66 = Eigen::Matrix<double, 6, 6>;

enum class NearestRotationMethod
{
    FOAM,
    SVD
};

constexpr double DEFAULT_RANK_TOLERANCE = 1e-7;
constexpr double DEFAULT_SQP_SQUARED_TOLERANCE = 1e-10;
constexpr double DEFAULT_SQP_DET_THRESHOLD = 1.001;
constexpr NearestRotationMethod DEFAULT_NEAREST_ROTATION_METHOD =
    NearestRotationMethod::SVD;
constexpr double DEFAULT_ORTHOGONALITY_SQUARED_ERROR_THRESHOLD = 1e-8;
constexpr double DEFAULT_EQUAL_VECTORS_SQUARED_DIFF = 1e-10;
constexpr double DEFAULT_EQUAL_SQUARED_ERRORS_DIFF = 1e-6;
constexpr double DEFAULT_POINT_VARIANCE_THRESHOLD = 1e-5;
const double SQRT3 = std::sqrt(3);

struct SQPSolution
{
    Vector9d r;     // Actual matrix upon convergence
    Vector9d r_hat; // "Clean" (nearest) rotation matrix
    Eigen::Vector3d t;
    int num_iterations;
    double sq_error;
};

void HandleSolution(const Matrix9d& Omega, const Eigen::Vector3d& point_mean,
                    SQPSolution& solution, SQPSolution* solutions,
                    double& min_sq_error, int& num_solutions);

double AverageSquaredProjectionError(
    const SQPSolution& solution,
    const std::vector<Eigen::Vector2d>& projections,
    const std::vector<Eigen::Vector3d>& points);

std::vector<double> AverageSquaredProjectionErrors(
    const std::vector<SQPSolution>& solutions,
    const std::vector<Eigen::Vector2d>& projections,
    const std::vector<Eigen::Vector3d>& points);

// Test cheirality for a given solution
bool TestPositiveDepth(const SQPSolution& solution,
                       const Eigen::Vector3d& point_mean);

// TODO: use Eign::determinant
inline double Determinant9x1(const Vector9d& r)
{
    return r[0] * r[4] * r[8] + r[1] * r[5] * r[6] + r[2] * r[3] * r[7] -
           r[6] * r[4] * r[2] - r[7] * r[5] * r[0] - r[8] * r[3] * r[1];
}

void SolveSQPSystem(const Vector9d& r, const Matrix9d& Omega, Vector9d& delta);

//
// Invert a 3x3 symmetrix matrix (using low triangle values only)
bool InvertSymmetric3x3(const Eigen::Matrix3d& Q, Eigen::Matrix3d& Qinv,
                        double det_threshold = 1e-8);

// Simple SVD - based nearest rotation matrix.
// Argument should be a ROW-MAJOR matrix representation.
// Returns a ROW-MAJOR vector representation of the nearest rotation matrix.
void findNearestRotationMatrixBySVD(const Vector9d& e, Vector9d& r);

// Faster nearest rotation computation based on FOAM. See docs for reference.
void findNearestRotationMatrixByFOAM(const Vector9d& e, Vector9d& r);

double OrthogonalityError(const Vector9d& a);

//
// Compute the 3D null space (N) and 6D normal space (H) of the constraint
// Jacobian at a 9D vector r (not necessarilly a rotation-yet it should be
// rank-3)
void RowAndNullSpace(const Vector9d& r, Matrix96& H, Matrix93& N, Matrix66& K,
                      double norm_threhsold = 0.1);

} // namespace tl
