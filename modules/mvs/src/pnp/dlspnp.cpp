#include "dlspnp.h"
#include "impl/dlspnp.hpp"

#include <cmath>

#include <Eigen/Eigenvalues>
#include <glog/logging.h>

namespace tl {

using internal::CreateMacaulayMatrix;
using internal::ExtractJacobianCoefficients;
using internal::LeftMultiplyMatrix;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using Matrix39d = Eigen::Matrix<double, 3, 9>;
using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Vector9d = Eigen::Matrix<double, 9, 1>;

bool DLSPnp(const std::vector<Eigen::Vector2d>& imagePoints,
            const std::vector<Eigen::Vector3d>& objectPoints,
            std::vector<Eigen::Quaterniond>& rotations,
            std::vector<Eigen::Vector3d>& translations)
{
    CHECK_GE(imagePoints.size(), 3);
    CHECK_EQ(imagePoints.size(), objectPoints.size());

    const size_t num_correspondences = imagePoints.size();

    // Holds the normalized feature positions cross multiplied with itself
    // i.e. n * n^t. This value is used multiple times so it is efficient to
    // pre-compute it.
    std::vector<Matrix3d> NNt;
    NNt.reserve(num_correspondences);
    for (const auto& imgPoint : imagePoints) {
        const Vector3d N = imgPoint.homogeneous().normalized();
        NNt.push_back(N * N.transpose());
    }

    // The bottom-right symmetric block matrix of inverse(A^T * A). Matrix H
    // from Eq. 25 in the Appendix of the DLS paper.
    Matrix3d h_inverse = num_correspondences * Matrix3d::Identity();
    for (size_t i{0}; i < num_correspondences; i++) {
        h_inverse -= NNt[i];
    }
    const Matrix3d h_matrix = h_inverse.inverse();

    // Compute V*W*b with the rotation parameters factored out. This is the
    // translation parameterized by the 9 entries of the rotation matrix.
    Matrix39d translation_factor = Matrix39d::Zero();
    for (size_t i{0}; i < num_correspondences; i++) {
        translation_factor =
            translation_factor + (NNt[i] - Matrix3d::Identity()) *
                                     LeftMultiplyMatrix(objectPoints[i]);
    }

    translation_factor = h_matrix * translation_factor;

    // Compute the cost function J' of Eq. 17 in DLS paper. This is a factorized
    // version where the rotation matrix parameters have been pulled out. The
    // entries to this equation are the coefficients to the cost function which
    // is a quartic in the rotation parameters.
    Matrix9d ls_cost_coefficients = Matrix9d::Zero();
    for (size_t i{0}; i < num_correspondences; i++) {
        ls_cost_coefficients +=

            (LeftMultiplyMatrix(objectPoints[i]) + translation_factor)
                .transpose() *
            (Matrix3d::Identity() - NNt[i]) *
            (LeftMultiplyMatrix(objectPoints[i]) + translation_factor);
    }

    // Extract the coefficients of the jacobian (Eq. 18) from the
    // ls_cost_coefficients matrix. The jacobian represent 3 monomials in the
    // rotation parameters. Each entry of the jacobian will be 0 at the roots of
    // the polynomial, so we can arrange a system of polynomials from these
    // equations.
    double f1_coeff[20];
    double f2_coeff[20];
    double f3_coeff[20];
    ExtractJacobianCoefficients(ls_cost_coefficients, f1_coeff, f2_coeff,
                                f3_coeff);

    // We create one equation with random terms that is generally non-zero at
    // the roots of our system.
    const Vector4d rand_vec = 100.0 * Vector4d::Random();
    const double macaulay_term[4] = {rand_vec(0), rand_vec(1), rand_vec(2),
                                     rand_vec(3)};

    // Create Macaulay matrix that will be used to solve our polynonomial
    // system.
    const MatrixXd& macaulay_matrix =
        CreateMacaulayMatrix(f1_coeff, f2_coeff, f3_coeff, macaulay_term);

    // Via the Schur complement trick, the top-left of the Macaulay matrix
    // contains a multiplication matrix whose eigenvectors correspond to
    // solutions to our system of equations.
    const MatrixXd solution_polynomial =
        macaulay_matrix.block<27, 27>(0, 0) -
        (macaulay_matrix.block<27, 93>(0, 27) *
         macaulay_matrix.block<93, 93>(27, 27).partialPivLu().solve(
             macaulay_matrix.block<93, 27>(27, 0)));

    // Extract eigenvectors of the solution polynomial to obtain the roots which
    // are contained in the entries of the eigenvectors.
    const Eigen::EigenSolver<MatrixXd> eigen_solver(solution_polynomial);

    // Many of the eigenvectors will contain complex solutions so we must filter
    // them to find the real solutions.
    const auto eigen_vectors = eigen_solver.eigenvectors();
    for (int i{0}; i < 27; i++) {
        // The first entry of the eigenvector should equal 1 according to our
        // polynomial, so we must divide each solution by the first entry.
        std::complex<double> s1 = eigen_vectors(9, i) / eigen_vectors(0, i);
        std::complex<double> s2 = eigen_vectors(3, i) / eigen_vectors(0, i);
        std::complex<double> s3 = eigen_vectors(1, i) / eigen_vectors(0, i);

        // If the rotation solutions are real, treat this as a valid candidate
        // rotation.
        constexpr double kEpsilon{1e-6};
        if (std::abs(s1.imag()) < kEpsilon && std::abs(s2.imag()) < kEpsilon &&
            std::abs(s3.imag()) < kEpsilon) {
            // Compute the rotation (which is the transpose rotation of our
            // solution) and translation.
            Quaterniond soln_rotation(1.0, s1.real(), s2.real(), s3.real());
            soln_rotation = soln_rotation.inverse().normalized();

            const Matrix3d rmat = soln_rotation.inverse().toRotationMatrix();
            const Eigen::Map<const Vector9d> rvec(rmat.data());
            const Vector3d soln_translation = translation_factor * rvec;

            // TODO: evaluate cost function and return it as an output variable.

            // Check that all points are in front of the camera.
            // Discard the solution if this is not the case.
            bool all_points_in_front_of_camera{true};
            for (size_t j{0}; j < num_correspondences; j++) {
                const Vector3d transformed_point =
                    soln_rotation * objectPoints[j] + soln_translation;
                if (transformed_point.z() < 0) {
                    all_points_in_front_of_camera = false;
                    break;
                }
            }

            if (all_points_in_front_of_camera) {
                rotations.push_back(soln_rotation);
                translations.push_back(soln_translation);
            }
        }
    }

    return !translations.empty();
}

} // namespace tl
