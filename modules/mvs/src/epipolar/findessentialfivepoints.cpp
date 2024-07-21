#include "findessentialfivepoints.h"

#include <Eigen/Eigenvalues>
#include <glog/logging.h>

namespace tl {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::RowVector3d;
using Eigen::RowVector4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Matrix10d = Eigen::Matrix<double, 10, 10>;
using Matrix10x20d = Eigen::Matrix<double, 10, 20>;
using Matrix94d = Eigen::Matrix<double, 9, 4>;
using Matrix9x20d = Eigen::Matrix<double, 9, 20>;
using MatrixX9d = Eigen::Matrix<double, Eigen::Dynamic, 9>;
using RowVector10d = Eigen::Matrix<double, 1, 10>;
using RowVector20d = Eigen::Matrix<double, 1, 20>;
using Vector9d = Eigen::Matrix<double, 9, 1>;

namespace {

// Multiply two 1D polynomials of variables x, y, z. e.g.
//     poly1 = a[0]x + a[1]y + a[2]z + a[3]
//     poly2 = b[0]x + b[1]y + b[2]z + b[3]
//     poly1 * poly2 = coeff * [xx xy+yx yy xz+zx yz+zy zz x y z 1]'
RowVector10d MultiplyDegOnePoly(const RowVector4d& a, const RowVector4d& b)
{
    RowVector10d out;
    // xx
    out(0) = a(0) * b(0);
    // xy + yx
    out(1) = a(0) * b(1) + a(1) * b(0);
    // yy
    out(2) = a(1) * b(1);
    // xz + zx
    out(3) = a(0) * b(2) + a(2) * b(0);
    // yz + zy
    out(4) = a(1) * b(2) + a(2) * b(1);
    // zz
    out(5) = a(2) * b(2);
    // x
    out(6) = a(0) * b(3) + a(3) * b(0);
    // y
    out(7) = a(1) * b(3) + a(3) * b(1);
    // z
    out(8) = a(2) * b(3) + a(3) * b(2);
    // 1
    out(9) = a(3) * b(3);
    return out;
}

// Multiply a 2D poly (in x, y, z) and a 1D poly in grevlex order.
// [ xxx xxy xyy yyy xxz xyz yyz xzz yzz zzz xx xy yy xz yz zz x y z 1 ]
RowVector20d MultiplyDegTwoDegOnePoly(const RowVector10d& a,
                                      const RowVector4d& b)
{
    RowVector20d out;
    // x^3
    out(0) = a(0) * b(0);
    // x^2y
    out(1) = a(0) * b(1) + a(1) * b(0);
    // xy^2
    out(2) = a(1) * b(1) + a(2) * b(0);
    // y^3
    out(3) = a(2) * b(1);
    // x^2z
    out(4) = a(0) * b(2) + a(3) * b(0);
    // xyz
    out(5) = a(1) * b(2) + a(3) * b(1) + a(4) * b(0);
    // y^2z
    out(6) = a(2) * b(2) + a(4) * b(1);
    // xz^2
    out(7) = a(3) * b(2) + a(5) * b(0);
    // yz^2
    out(8) = a(4) * b(2) + a(5) * b(1);
    // z^3
    out(9) = a(5) * b(2);
    // x^2
    out(10) = a(0) * b(3) + a(6) * b(0);
    // xy
    out(11) = a(1) * b(3) + a(6) * b(1) + a(7) * b(0);
    // y^2
    out(12) = a(2) * b(3) + a(7) * b(1);
    // xz
    out(13) = a(3) * b(3) + a(6) * b(2) + a(8) * b(0);
    // yz
    out(14) = a(4) * b(3) + a(7) * b(2) + a(8) * b(1);
    // z^2
    out(15) = a(5) * b(3) + a(8) * b(2);
    // x
    out(16) = a(6) * b(3) + a(9) * b(0);
    // y
    out(17) = a(7) * b(3) + a(9) * b(1);
    // z
    out(18) = a(8) * b(3) + a(9) * b(2);
    // 1
    out(19) = a(9) * b(3);
    return out;
}

RowVector20d GetDeterminantConstraint(const Eigen::RowVector4d nullspace[3][3])
{
    // Singularity constraint.
    const RowVector20d determinant =
        MultiplyDegTwoDegOnePoly(
            MultiplyDegOnePoly(nullspace[0][1], nullspace[1][2]) -
                MultiplyDegOnePoly(nullspace[0][2], nullspace[1][1]),
            nullspace[2][0]) +
        MultiplyDegTwoDegOnePoly(
            MultiplyDegOnePoly(nullspace[0][2], nullspace[1][0]) -
                MultiplyDegOnePoly(nullspace[0][0], nullspace[1][2]),
            nullspace[2][1]) +
        MultiplyDegTwoDegOnePoly(
            MultiplyDegOnePoly(nullspace[0][0], nullspace[1][1]) -
                MultiplyDegOnePoly(nullspace[0][1], nullspace[1][0]),
            nullspace[2][2]);
    return determinant;
}

inline RowVector10d EEt(const Eigen::RowVector4d nullspace[3][3], int i, int j)
{
    return MultiplyDegOnePoly(nullspace[i][0], nullspace[j][0]) +
           MultiplyDegOnePoly(nullspace[i][1], nullspace[j][1]) +
           MultiplyDegOnePoly(nullspace[i][2], nullspace[j][2]);
}

// Builds the trace constraint: EEtE - 1/2 trace(EEt)E = 0
Matrix9x20d GetTraceConstraint(const Eigen::RowVector4d nullspace[3][3])
{
    // Comput EEt.
    RowVector10d eet[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            eet[i][j] = 2 * EEt(nullspace, i, j);
        }
    }

    // Compute the trace.
    const RowVector10d trace = eet[0][0] + eet[1][1] + eet[2][2];

    // Multiply EEt with E.
    Matrix9x20d constraint;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            constraint.row(3 * i + j) =
                MultiplyDegTwoDegOnePoly(eet[i][0], nullspace[0][j]) +
                MultiplyDegTwoDegOnePoly(eet[i][1], nullspace[1][j]) +
                MultiplyDegTwoDegOnePoly(eet[i][2], nullspace[2][j]) -
                0.5 * MultiplyDegTwoDegOnePoly(trace, nullspace[i][j]);
        }
    }

    return constraint;
}

Matrix10x20d BuildConstraintMatrix(const Eigen::RowVector4d nullspace[3][3])
{
    Matrix10x20d matrix;
    matrix.block<9, 20>(0, 0) = GetTraceConstraint(nullspace);
    matrix.row(9) = GetDeterminantConstraint(nullspace);
    return matrix;
}

} // namespace

bool FivePointRelativePose(const std::vector<Eigen::Vector2d>& imgPoints1,
                           const std::vector<Eigen::Vector2d>& imgPoints2,
                           std::vector<Eigen::Matrix3d>* ematrices)
{
    return FindEssentialFivePoints(imgPoints1, imgPoints2, ematrices);
}

bool FindEssentialFivePoints(const std::vector<Eigen::Vector2d>& imgPoints1,
                             const std::vector<Eigen::Vector2d>& imgPoints2,
                             std::vector<Eigen::Matrix3d>* ematrices)
{
    constexpr size_t kMinNumPoints = 5;

    CHECK_EQ(imgPoints1.size(), imgPoints2.size());
    CHECK_GE(imgPoints1.size(), kMinNumPoints)
        << "Require at least 5 correspondences to perform the 5 point "
           "essential matrix algorithm.";

    // Step 1. Create the nx9 matrix containing epipolar constraints.
    // Essential matrix is a linear combination of the 4 vectors spanning the
    // null space of this matrix.
    MatrixX9d epipolar_constraint(imgPoints1.size(), 9);
    for (int i = 0; i < imgPoints1.size(); i++) {
        // Fill matrix with the epipolar constraint from q2'*E*q1 = 0.
        // clang-format off
        epipolar_constraint.row(i) <<
            imgPoints2[i].x() * imgPoints1[i].x(),
            imgPoints2[i].y() * imgPoints1[i].x(),
            imgPoints1[i].x(),
            imgPoints2[i].x() * imgPoints1[i].y(),
            imgPoints2[i].y() * imgPoints1[i].y(),
            imgPoints1[i].y(),
            imgPoints2[i].x(),
            imgPoints2[i].y(),
            1.;
        // clang-format on
    }

    Matrix94d nullspace;

    // Extract the null space from a minimal sampling (using LU) or non-minimal
    // sampling (using SVD).
    if (imgPoints1.size() == kMinNumPoints) {
        const Eigen::FullPivLU<MatrixXd> lu{epipolar_constraint};
        if (lu.dimensionOfKernel() != 4) {
            return false;
        }
        nullspace = lu.kernel();
    }
    else {
        const Eigen::JacobiSVD<MatrixXd> svd{
            epipolar_constraint.transpose() * epipolar_constraint,
            Eigen::ComputeFullV};
        nullspace = svd.matrixV().rightCols<4>();
    }

    const RowVector4d nullspace_matrix[3][3]{
        {nullspace.row(0), nullspace.row(3), nullspace.row(6)},
        {nullspace.row(1), nullspace.row(4), nullspace.row(7)},
        {nullspace.row(2), nullspace.row(5), nullspace.row(8)}};

    // Step 2. Expansion of the epipolar constraints on the determinant and
    // trace.
    const Matrix10x20d constraint_matrix =
        BuildConstraintMatrix(nullspace_matrix);

    // Step 3. Eliminate part of the matrix to isolate polynomials in z.
    Eigen::FullPivLU<Matrix10d> c_lu{constraint_matrix.block<10, 10>(0, 0)};
    Matrix10d eliminated_matrix =
        c_lu.solve(constraint_matrix.block<10, 10>(0, 10));

    Matrix10d action_matrix = Matrix10d::Zero();
    action_matrix.block<3, 10>(0, 0) = eliminated_matrix.block<3, 10>(0, 0);
    action_matrix.row(3) = eliminated_matrix.row(4);
    action_matrix.row(4) = eliminated_matrix.row(5);
    action_matrix.row(5) = eliminated_matrix.row(7);
    action_matrix(6, 0) = -1.;
    action_matrix(7, 1) = -1.;
    action_matrix(8, 3) = -1.;
    action_matrix(9, 6) = -1.;

    Eigen::EigenSolver<Matrix10d> eigensolver{action_matrix};
    const auto& eigenvectors = eigensolver.eigenvectors();
    const auto& eigenvalues = eigensolver.eigenvalues();

    // Now that we have x, y, and z we need to substitute them back into the
    // null space to get a valid essential matrix solution.
    for (int i = 0; i < 10; i++) {
        // Only consider real solutions.
        if (eigenvalues(i).imag() != 0) {
            continue;
        }

        Matrix3d E;
        Eigen::Map<Vector9d>(E.data()) =
            nullspace * eigenvectors.col(i).tail<4>().real();
        ematrices->emplace_back(E);
    }

    return !ematrices->empty();
}

} // namespace tl
