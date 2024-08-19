#include "p5pfocalrd.h"

#include <glog/logging.h>

#include <tMath/Eigen/Types>
#include <tMath/Polynomial>

namespace tl {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Matrix5d = Eigen::Matrix<double, 5, 5>;
using Matrix58d = Eigen::Matrix<double, 5, 8>;
using Matrix83d = Eigen::Matrix<double, 8, 3>;
using Vector5d = Eigen::Vector<double, 5>;

namespace {
// Helper function which takes in the null basis (three 8-dimension vectors)
// which has two unknowns and solves for one of them using Sylvester matrix
// computed from the orthonormal constraint on rows 1 and 2 of the projection
// matrix. See Eq 10, 11 in the paper for details. This has been precomputed
// with matlab for optimal runtime.
void SetupAndSolveSylvesterMatrix(const Matrix83d& n, double* y1_soln,
                                  double* y2_soln)
{
    static const double kTolerance = 1e-12;

    // The Sylvester matrix will help us solve for one of the two unknown
    // variables. By using Eq 10, 11 (i.e., the orthonormal constraint), we can
    // set up the two equations as a polynomial in y1, leaving y2 as part of the
    // coefficients of the polynomial in y1. These variables are the
    // coefficients of y2 for each coefficient of y1. We compact them for
    // simplicity so that the quartic equation below is not 1 million lines
    // long.
    const double s11_1 =
        n(0, 0) * n(4, 0) + n(1, 0) * n(5, 0) + n(2, 0) * n(6, 0);
    const double s12_2 = n(0, 0) * n(4, 1) + n(0, 1) * n(4, 0) +
                         n(1, 0) * n(5, 1) + n(1, 1) * n(5, 0) +
                         n(2, 0) * n(6, 1) + n(2, 1) * n(6, 0);
    const double s12_1 = n(0, 0) * n(4, 2) + n(0, 2) * n(4, 0) +
                         n(1, 0) * n(5, 2) + n(1, 2) * n(5, 0) +
                         n(2, 0) * n(6, 2) + n(2, 2) * n(6, 0);
    const double s13_3 =
        n(0, 1) * n(4, 1) + n(1, 1) * n(5, 1) + n(2, 1) * n(6, 1);
    const double s13_2 = n(0, 1) * n(4, 2) + n(0, 2) * n(4, 1) +
                         n(1, 1) * n(5, 2) + n(1, 2) * n(5, 1) +
                         n(2, 1) * n(6, 2) + n(2, 2) * n(6, 1);
    const double s13_1 =
        n(0, 2) * n(4, 2) + n(1, 2) * n(5, 2) + n(2, 2) * n(6, 2);
    const double s21_1 = n(0, 0) * n(0, 0) + n(1, 0) * n(1, 0) +
                         n(2, 0) * n(2, 0) - n(4, 0) * n(4, 0) -
                         n(5, 0) * n(5, 0) - n(6, 0) * n(6, 0);
    const double s22_2 = n(0, 0) * n(0, 1) * 2.0 + n(1, 0) * n(1, 1) * 2.0 +
                         n(2, 0) * n(2, 1) * 2.0 - n(4, 0) * n(4, 1) * 2.0 -
                         n(5, 0) * n(5, 1) * 2.0 - n(6, 0) * n(6, 1) * 2.0;
    const double s22_1 = n(0, 0) * n(0, 2) * 2.0 + n(1, 0) * n(1, 2) * 2.0 +
                         n(2, 0) * n(2, 2) * 2.0 - n(4, 0) * n(4, 2) * 2.0 -
                         n(5, 0) * n(5, 2) * 2.0 - n(6, 0) * n(6, 2) * 2.0;
    const double s23_3 = n(0, 1) * n(0, 1) + n(1, 1) * n(1, 1) +
                         n(2, 1) * n(2, 1) - n(4, 1) * n(4, 1) -
                         n(5, 1) * n(5, 1) - n(6, 1) * n(6, 1);
    const double s23_2 = n(0, 1) * n(0, 2) * 2.0 + n(1, 1) * n(1, 2) * 2.0 +
                         n(2, 1) * n(2, 2) * 2.0 - n(4, 1) * n(4, 2) * 2.0 -
                         n(5, 1) * n(5, 2) * 2.0 - n(6, 1) * n(6, 2) * 2.0;
    const double s23_1 = n(0, 2) * n(0, 2) + n(1, 2) * n(1, 2) +
                         n(2, 2) * n(2, 2) - n(4, 2) * n(4, 2) -
                         n(5, 2) * n(5, 2) - n(6, 2) * n(6, 2);

    // Setting the determinant of the Sylvester matrix to 0 will create a
    // quartic polynomial in y2. The roots of this polynomial are the solutions
    // to y2.
    VectorXd coeffs(5);
    coeffs(0) =
        (s11_1 * s11_1) * (s23_3 * s23_3) + (s13_3 * s13_3) * (s21_1 * s21_1) +
        s11_1 * s13_3 * (s22_2 * s22_2) + (s12_2 * s12_2) * s21_1 * s23_3 -
        s11_1 * s12_2 * s22_2 * s23_3 - s11_1 * s13_3 * s21_1 * s23_3 * 2.0 -
        s12_2 * s13_3 * s21_1 * s22_2;
    coeffs(1) =
        s11_1 * s13_2 * (s22_2 * s22_2) +
        s13_2 * s13_3 * (s21_1 * s21_1) * 2.0 +
        (s12_2 * s12_2) * s21_1 * s23_2 +
        (s11_1 * s11_1) * s23_2 * s23_3 * 2.0 - s11_1 * s12_1 * s22_2 * s23_3 -
        s11_1 * s12_2 * s22_1 * s23_3 - s11_1 * s12_2 * s22_2 * s23_2 -
        s11_1 * s13_2 * s21_1 * s23_3 * 2.0 -
        s11_1 * s13_3 * s21_1 * s23_2 * 2.0 +
        s11_1 * s13_3 * s22_1 * s22_2 * 2.0 +
        s12_1 * s12_2 * s21_1 * s23_3 * 2.0 - s12_1 * s13_3 * s21_1 * s22_2 -
        s12_2 * s13_2 * s21_1 * s22_2 - s12_2 * s13_3 * s21_1 * s22_1;
    coeffs(2) =
        (s11_1 * s11_1) * (s23_2 * s23_2) + (s13_2 * s13_2) * (s21_1 * s21_1) +
        s11_1 * s13_1 * (s22_2 * s22_2) + s11_1 * s13_3 * (s22_1 * s22_1) +
        s13_1 * s13_3 * (s21_1 * s21_1) * 2.0 +
        (s12_2 * s12_2) * s21_1 * s23_1 + (s12_1 * s12_1) * s21_1 * s23_3 +
        (s11_1 * s11_1) * s23_1 * s23_3 * 2.0 - s11_1 * s12_1 * s22_1 * s23_3 -
        s11_1 * s12_1 * s22_2 * s23_2 - s11_1 * s12_2 * s22_1 * s23_2 -
        s11_1 * s12_2 * s22_2 * s23_1 - s11_1 * s13_1 * s21_1 * s23_3 * 2.0 -
        s11_1 * s13_2 * s21_1 * s23_2 * 2.0 +
        s11_1 * s13_2 * s22_1 * s22_2 * 2.0 -
        s11_1 * s13_3 * s21_1 * s23_1 * 2.0 +
        s12_1 * s12_2 * s21_1 * s23_2 * 2.0 - s12_1 * s13_2 * s21_1 * s22_2 -
        s12_1 * s13_3 * s21_1 * s22_1 - s12_2 * s13_1 * s21_1 * s22_2 -
        s12_2 * s13_2 * s21_1 * s22_1;
    coeffs(3) =
        s11_1 * s13_2 * (s22_1 * s22_1) +
        s13_1 * s13_2 * (s21_1 * s21_1) * 2.0 +
        (s12_1 * s12_1) * s21_1 * s23_2 +
        (s11_1 * s11_1) * s23_1 * s23_2 * 2.0 - s11_1 * s12_1 * s22_1 * s23_2 -
        s11_1 * s12_1 * s22_2 * s23_1 - s11_1 * s12_2 * s22_1 * s23_1 -
        s11_1 * s13_1 * s21_1 * s23_2 * 2.0 +
        s11_1 * s13_1 * s22_1 * s22_2 * 2.0 -
        s11_1 * s13_2 * s21_1 * s23_1 * 2.0 +
        s12_1 * s12_2 * s21_1 * s23_1 * 2.0 - s12_1 * s13_1 * s21_1 * s22_2 -
        s12_1 * s13_2 * s21_1 * s22_1 - s12_2 * s13_1 * s21_1 * s22_1;
    coeffs(4) =
        (s11_1 * s11_1) * (s23_1 * s23_1) + (s13_1 * s13_1) * (s21_1 * s21_1) +
        s11_1 * s13_1 * (s22_1 * s22_1) + (s12_1 * s12_1) * s21_1 * s23_1 -
        s11_1 * s12_1 * s22_1 * s23_1 - s11_1 * s13_1 * s21_1 * s23_1 * 2.0 -
        s12_1 * s13_1 * s21_1 * s22_1;

    // Solve Quartic
    VectorXd roots;
    CHECK(ceres::internal::FindPolynomialRoots(coeffs, &roots, nullptr))
        << "Quartic could not be solved for p5pfr.";

    // Solve for y1 by substituting y2 solutions back into Eq 10, 11.
    for (int i = 0; i < 4; i++) {
        // Substituting solutions for y2 yields a linear equation of the form
        // ax + b = 0.
        double a = (s22_2 - (s12_2 * s21_1) / s11_1) * roots[i] + s22_1 -
                   (s12_1 * s21_1) / s11_1;

        double b = (s23_3 - (s13_3 * s21_1) / s11_1) * roots[i] * roots[i] +
                   (s23_2 - (s13_2 * s21_1) / s11_1) * roots[i] + s23_1 -
                   (s13_1 * s21_1) / s11_1;

        // If dividing by a is unstable, recompute a and b.
        if (a < kTolerance) {
            a = (s12_2 - (s22_2 * s11_1) / s21_1) * roots[i] + s12_1 -
                (s22_1 * s11_1) / s21_1;

            b = (s13_3 - (s23_3 * s11_1) / s21_1) * roots[i] * roots[i] +
                (s13_2 - (s23_2 * s11_1) / s21_1) * roots[i] + s13_1 -
                (s23_1 * s11_1) / s21_1;
        }

        y1_soln[i] = -b / a;
        y2_soln[i] = roots[i];
    }
}
} // namespace

bool FivePointFocalLengthRadialDistortion(
    const std::vector<Eigen::Vector2d>& feature_vectors,
    const std::vector<Eigen::Vector3d>& world_points,
    const int num_radial_distortion_params,
    std::vector<Matrix34d>* projection_matrices,
    std::vector<std::vector<double>>* radial_distortions)
{
    CHECK_EQ(feature_vectors.size(), 5);
    CHECK_EQ(world_points.size(), 5);
    CHECK_LE(num_radial_distortion_params, 3);
    CHECK_GT(num_radial_distortion_params, 0);

    // Set up 5x8 constraint from row3 of the projection constraint in Eq 7.
    Matrix58d row3_constraint;
    for (int i = 0; i < 5; i++) {
        row3_constraint.row(i) << -feature_vectors[i].y() *
                                      world_points[i].homogeneous().transpose(),
            feature_vectors[i].x() * world_points[i].homogeneous().transpose();
    }

    // Compute nullspace basis of the constraint to parameterize the first two
    // rows of the projection matrix in terms of 2 unknowns.
    const Matrix83d projrow12_basis = row3_constraint.fullPivLu().kernel();

    // Create Sylvester matrix and solve for one of the unknowns.
    double y1_solution[4];
    double y2_solution[4];
    SetupAndSolveSylvesterMatrix(projrow12_basis, y1_solution, y2_solution);

    // Loop over all possible value of y1, y2.
    for (int i = 0; i < 4; i++) {
        // y1 and y2 specify a candidate solution to the first two rows of the
        // projection matrix (up to scale). Set those rows here.
        Eigen::Matrix<double, 3, 4, Eigen::RowMajor> candidate_proj =
            Eigen::Matrix<double, 3, 4, Eigen::RowMajor>::Zero();
        Eigen::Map<Eigen::Matrix<double, 8, 1>> candidate_proj_map(
            candidate_proj.data());
        candidate_proj_map =
            projrow12_basis * Vector3d(y1_solution[i], y2_solution[i], 1.0);

        // Solve for p31, p32, p33 in terms of an unknown delta. This will later
        // be scaled once we solve for delta in the linear system below.
        const double denominator = candidate_proj(0, 0) * candidate_proj(1, 1) -
                                   candidate_proj(0, 1) * candidate_proj(1, 0);
        candidate_proj.block<1, 3>(2, 0)
            << (candidate_proj(0, 1) * candidate_proj(1, 2) -
                candidate_proj(0, 2) * candidate_proj(1, 1)) /
                   denominator,
            -(candidate_proj(0, 0) * candidate_proj(1, 2) -
              candidate_proj(0, 2) * candidate_proj(1, 0)) /
                denominator,
            1.0;

        // Set up constraints from Eq 6, 12, and 13. This equation is set up as
        // an Ax = b type of problem and solved in a least squares sense.
        Matrix5d row12_constraint;
        Vector5d row12_constraint_soln;
        for (int j = 0; j < 5; j++) {
            // Precompute some of the coefficients.
            const double r_sq = feature_vectors[j].squaredNorm();
            const double z_projection =
                candidate_proj.row(2).head<3>().dot(world_points[j]);

            // If the y image coordinate is too small, use the equation from the
            // second row.
            const double eps = 0.1;
            if (std::abs(feature_vectors[j].y()) < eps) {
                const double x_projection =
                    candidate_proj.row(0).dot(world_points[j].homogeneous());
                row12_constraint.row(j)
                    << -feature_vectors[j].x() * z_projection,
                    -feature_vectors[j].x(), x_projection * r_sq,
                    x_projection * r_sq * r_sq,
                    x_projection * r_sq * r_sq * r_sq;
                row12_constraint_soln[j] = -x_projection;
            }
            else {
                const double y_projection =
                    -candidate_proj.row(1).dot(world_points[j].homogeneous());
                row12_constraint.row(j)
                    << feature_vectors[j].y() * z_projection,
                    feature_vectors[j].y(), y_projection * r_sq,
                    y_projection * r_sq * r_sq,
                    y_projection * r_sq * r_sq * r_sq;
                row12_constraint_soln[j] = -y_projection;
            }
        }

        double delta;
        double p34;
        std::vector<double> rad_dist_soln;

        // Solve our Ax = b equation to yield k1, w, p34.
        if (num_radial_distortion_params == 3) {
            const Vector5d unknowns_soln =
                row12_constraint.colPivHouseholderQr().solve(
                    row12_constraint_soln);
            delta = unknowns_soln(0);
            p34 = unknowns_soln(1);
            // Set output distortion params.
            rad_dist_soln.push_back(unknowns_soln[2]);
            rad_dist_soln.push_back(unknowns_soln[3]);
            rad_dist_soln.push_back(unknowns_soln[4]);
        }
        else if (num_radial_distortion_params == 2) {
            const Vector4d unknowns_soln =
                row12_constraint.leftCols<4>().colPivHouseholderQr().solve(
                    row12_constraint_soln);
            delta = unknowns_soln(0);
            p34 = unknowns_soln(1);
            // Set output distortion params.
            rad_dist_soln.push_back(unknowns_soln[2]);
            rad_dist_soln.push_back(unknowns_soln[3]);
        }
        else {
            const Vector3d unknowns_soln =
                row12_constraint.leftCols<3>().colPivHouseholderQr().solve(
                    row12_constraint_soln);
            delta = unknowns_soln(0);
            p34 = unknowns_soln(1);
            // Set output distortion params.
            rad_dist_soln.push_back(unknowns_soln[2]);
        }

        // Set the last row of the projection matrix.
        candidate_proj.block<1, 3>(2, 0) *= delta;
        candidate_proj(2, 3) = p34;

        // Set output projection matrix.
        projection_matrices->push_back(candidate_proj);

        // Output radial distortions.
        radial_distortions->push_back(rad_dist_soln);
    }

    return true;
}

} // namespace tl
