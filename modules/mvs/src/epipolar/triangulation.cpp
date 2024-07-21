#include "triangulation.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <glog/logging.h>

#include <tCore/Math>
#include <tMvs/Feature>

#include "basics.h"

namespace tl {

namespace {

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using Matrix23d = Eigen::Matrix<double, 2, 3>;

// Given either a fundamental or essential matrix and two corresponding images
// points such that the corrected image points (cpoint1, cpoint2) minimize
//         cpoint1' * E * cpoint2.
void FindOptimalImagePoints(const Eigen::Matrix3d& E,
                            const Eigen::Vector2d& point1,
                            const Eigen::Vector2d& point2,
                            Eigen::Vector2d* cpoint1, Eigen::Vector2d* cpoint2)
{
    const Vector3d pt1 = point1.homogeneous();
    const Vector3d pt2 = point2.homogeneous();

    // A helper matrix to isolate certain coordinates.
    Matrix23d s_matrix;
    // clang-format off
    s_matrix << 1., 0., 0.,
                0., 1., 0.;
    // clang-format on

    const Matrix2d e_submatrix = E.topLeftCorner<2, 2>();

    // The epipolar line from one image point in the other image.
    Vector2d epiline1 = s_matrix * E * pt2;
    Vector2d epiline2 = s_matrix * E.transpose() * pt1;

    const double a = epiline1.transpose() * e_submatrix * epiline2;
    const double b = (epiline1.squaredNorm() + epiline2.squaredNorm()) * 0.5;
    const double c = pt1.transpose() * E * pt2;
    const double d = std::sqrt(b * b - a * c);

    double lambda = c / (b + d);
    epiline1 -= e_submatrix * lambda * epiline1;
    epiline2 -= e_submatrix.transpose() * lambda * epiline2;

    lambda *= (2. * d) / (epiline1.squaredNorm() + epiline2.squaredNorm());

    *cpoint1 = (pt1 - s_matrix.transpose() * lambda * epiline1).hnormalized();
    *cpoint2 = (pt2 - s_matrix.transpose() * lambda * epiline2).hnormalized();
}

} // namespace

bool Triangulate(const Matrix34d& pose1, const Matrix34d& pose2,
                 const Eigen::Vector2d& pixel1, const Eigen::Vector2d& pixel2,
                 Eigen::Vector4d* point)
{
    Matrix3d E;
    EssentialMatrixFromTwoProjectionMatrices(pose1, pose2, &E);

    Vector2d cpoint1, cpoint2;
    FindOptimalImagePoints(E, pixel1, pixel2, &cpoint1, &cpoint2);

    // Now the two points are guaranteed to intersect. We can use the DLT method
    // since it is easy to construct.
    return TriangulateDLT(pose1, pose2, cpoint1, cpoint2, point);
}

bool TriangulateMidpoint(const std::vector<Eigen::Vector3d>& origins,
                         const std::vector<Eigen::Vector3d>& directions,
                         Eigen::Vector4d* point)
{
    CHECK_NOTNULL(point);
    CHECK_GE(origins.size(), 2);
    CHECK_EQ(origins.size(), directions.size());

    Matrix4d A = Matrix4d::Zero();
    Vector4d b = Vector4d::Zero();
    for (int i = 0; i < origins.size(); i++) {
        Vector4d dir;
        dir << directions[i], 0.;

        const Matrix4d A_tmp = Matrix4d::Identity() - dir * dir.transpose();
        A += A_tmp;
        b += A_tmp * origins[i].homogeneous();
    }

    Eigen::LLT<Matrix4d> solver{A};
    if (solver.info() != Eigen::Success) {
        return false;
    }

    *point = solver.solve(b);
    return true;
}

bool TriangulateDLT(const Matrix34d& pose1, const Matrix34d& pose2,
                    const Eigen::Vector2d& point1,
                    const Eigen::Vector2d& point2,
                    Eigen::Vector4d* triangulated_point)
{
    Matrix4d A;
    A.row(0) = point1[0] * pose1.row(2) - pose1.row(0);
    A.row(1) = point1[1] * pose1.row(2) - pose1.row(1);
    A.row(2) = point2[0] * pose2.row(2) - pose2.row(0);
    A.row(3) = point2[1] * pose2.row(2) - pose2.row(1);

    // Extract nullspace.
    *triangulated_point =
        A.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    return true;
}

bool triangulateTwoViews(const Matrix34d& pose1, const Matrix34d& pose2,
                         const Eigen::Vector2d& pixel1,
                         const Eigen::Vector2d& pixel2,
                         TriangulationMethodType type, Eigen::Vector4d* point)
{
    switch (type) {
        case TriangulationMethodType::STANDARD:
            return Triangulate(pose1, pose2, pixel1, pixel2, point);
        case TriangulationMethodType::DLT:
            return TriangulateDLT(pose1, pose2, pixel1, pixel2, point);
        case TriangulationMethodType::MIDPOINT: {
            std::vector<Vector3d> origins;
            std::vector<Vector3d> directions;

            const Matrix3d R1 = pose1.block<3, 3>(0, 0);
            origins.emplace_back(-R1.transpose() * pose1.col(3));
            directions.emplace_back(
                (R1.transpose() * pixel1.homogeneous()).normalized());
            const Matrix3d R2 = pose2.block<3, 3>(0, 0);
            origins.emplace_back(-R2.transpose() * pose2.col(3));
            directions.emplace_back(
                (R2.transpose() * pixel2.homogeneous()).normalized());
            return TriangulateMidpoint(origins, directions, point);
        }
        default:
            LOG(ERROR) << "Incompatible Triangulation type!";
            break;
    }

    return false;
}

bool TriangulateNViewSVD(const std::vector<Matrix34d>& poses,
                         const std::vector<Eigen::Vector2d>& points,
                         Eigen::Vector4d* triangulated_point)
{
    CHECK_EQ(poses.size(), points.size());

    MatrixXd A(3 * points.size(), 4 + points.size());
    for (int i = 0; i < points.size(); i++) {
        A.block<3, 4>(3 * i, 0) = -poses[i].matrix();
        A.block<3, 1>(3 * i, 4 + i) = points[i].homogeneous();
    }

    *triangulated_point =
        A.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>().head(4);
    return true;
}

bool TriangulateNView(const std::vector<Matrix34d>& poses,
                      const std::vector<Eigen::Vector2d>& points,
                      Eigen::Vector4d* triangulated_point)
{
    CHECK_EQ(poses.size(), points.size());

    Matrix4d A = Matrix4d::Zero();
    for (int i = 0; i < points.size(); i++) {
        const Vector3d norm_point = points[i].homogeneous().normalized();
        const Matrix34d cost_term =
            poses[i].matrix() -
            norm_point * norm_point.transpose() * poses[i].matrix();
        A = A + cost_term.transpose() * cost_term;
    }

    Eigen::SelfAdjointEigenSolver<Matrix4d> eigen_solver{A};
    *triangulated_point = eigen_solver.eigenvectors().col(0);
    return eigen_solver.info() == Eigen::Success;
}

bool IsTriangulatedPointInFrontOfCameras(const Feature2D2D& corr,
                                         const Eigen::Matrix3d& R,
                                         const Eigen::Vector3d& t)
{
    const Vector3d dir1 = corr.feature1.pos.homogeneous();
    const Vector3d dir2 = R.transpose() * corr.feature2.pos.homogeneous();

    const double dir1_dir2 = dir1.dot(dir2);
    const double dir1_pos = dir1.dot(t);
    const double dir2_pos = dir2.dot(t);

    return (dir2.squaredNorm() * dir1_pos - dir1_dir2 * dir2_pos > 0 &&
            dir1_dir2 * dir1_pos - dir1.squaredNorm() * dir2_pos > 0);
}

bool SufficientTriangulationAngle(
    const std::vector<Eigen::Vector3d>& directions, double minAngle)
{
    // Test that the angle between the rays is sufficient.
    const auto cosMinAngle = std::cos(math::degToRad(minAngle));
    for (size_t i{0}; i < directions.size(); ++i) {
        for (size_t j = i + 1; j < directions.size(); ++j) {
            if (directions[i].dot(directions[j]) < cosMinAngle) {
                return true;
            }
        }
    }
    return false;
}

} // namespace tl
