#include "p4p_focal.h"
#include "impl/p4p_focal_impl.h"

namespace thoht {

namespace {

void GetRigidTransform(const Matrix34d& points1, const Matrix34d& points2,
                       bool left_handed_coordinates, Eigen::Matrix3d& rotation,
                       Eigen::Vector3d& translation)
{
    // Move the centroid to the origin.
    const Eigen::Vector3d points1_mean = points1.rowwise().mean();
    const Eigen::Vector3d points2_mean = points2.rowwise().mean();

    const Matrix34d points1_centered = points1.colwise() - points1_mean;
    const Matrix34d points2_centered = points2.colwise() - points2_mean;

    // Normalize to unit size.
    const Matrix34d points1_norm = points1_centered.colwise().normalized();
    const Matrix34d points2_norm = points2_centered.colwise().normalized();

    // Compute the necessary rotation from the difference in points.
    Eigen::Matrix3d rotation_diff = points2_norm * points1_norm.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        rotation_diff, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& U = svd.matrixU();
    const auto& V_t = svd.matrixV().transpose();
    const auto& singularValues = svd.singularValues();

    Eigen::Matrix3d s = Eigen::Matrix3d::Zero();
    s(0, 0) = singularValues(0) < 0 ? -1.0 : 1.0;
    s(1, 1) = singularValues(1) < 0 ? -1.0 : 1.0;
    const double sign = (U * V_t).determinant() < 0 ? -1.0 : 1.0;
    s(2, 2) = left_handed_coordinates ? -sign : sign;

    rotation = U * s * V_t;
    translation = -rotation * points1_mean + points2_mean;
}

} // namespace

int FourPointPoseAndFocalLength(const Vector2dList& image_points,
                                const Vector3dList& world_points,
                                std::vector<Matrix34d>& projection_matrices)
{
    Eigen::Map<const Matrix24d> point2(image_points[0].data());
    Eigen::Map<const Matrix34d> point3(world_points[0].data());

    // Normalize the points such that the mean = 0, variance = sqrt(2.0).
    const Eigen::Vector3d point3_mean = point3.rowwise().mean();
    Matrix34d point3_norm = point3.colwise() - point3_mean;
    const double point3_var = point3_norm.colwise().norm().mean();
    point3_norm /= point3_var;

    // Scale 2D data so variance = sqrt(2.0).
    const double point2_var = point2.colwise().norm().mean();
    Matrix24d point2_norm = point2 / point2_var;

    // Precompute monomials.
    const double glab = (point3_norm.col(0) - point3_norm.col(1)).squaredNorm();
    const double glac = (point3_norm.col(0) - point3_norm.col(2)).squaredNorm();
    const double glad = (point3_norm.col(0) - point3_norm.col(3)).squaredNorm();
    const double glbc = (point3_norm.col(1) - point3_norm.col(2)).squaredNorm();
    const double glbd = (point3_norm.col(1) - point3_norm.col(3)).squaredNorm();
    const double glcd = (point3_norm.col(2) - point3_norm.col(3)).squaredNorm();

    if (glab * glac * glad * glbc * glbd * glcd < 1e-15) {
        return -1;
    }

    // Call the helper function.
    std::vector<double> focal_lengths;
    std::vector<Eigen::Vector3d> depths;
    FourPointFocalLengthHelper(glab, glac, glad, glbc, glbd, glcd, point2_norm,
                               &focal_lengths, &depths);

    if (focal_lengths.empty()) {
        return -1;
    }

    // Get the rotation and translation.
    for (int i = 0; i < focal_lengths.size(); i++) {
        // Create world points in camera coordinate system.
        Matrix34d point3_adjust;
        point3_adjust.block<2, 4>(0, 0) = point2_norm;
        point3_adjust.row(2).setConstant(focal_lengths[i]);
        point3_adjust.col(1) *= depths[i].x();
        point3_adjust.col(2) *= depths[i].y();
        point3_adjust.col(3) *= depths[i].z();

        // Fix the scale.
        Eigen::Matrix<double, 6, 1> d;
        d(0) = sqrt(
            glab / (point3_adjust.col(0) - point3_adjust.col(1)).squaredNorm());
        d(1) = sqrt(
            glac / (point3_adjust.col(0) - point3_adjust.col(2)).squaredNorm());
        d(2) = sqrt(
            glad / (point3_adjust.col(0) - point3_adjust.col(3)).squaredNorm());
        d(3) = sqrt(
            glbc / (point3_adjust.col(1) - point3_adjust.col(2)).squaredNorm());
        d(4) = sqrt(
            glbd / (point3_adjust.col(1) - point3_adjust.col(3)).squaredNorm());
        d(5) = sqrt(
            glcd / (point3_adjust.col(2) - point3_adjust.col(3)).squaredNorm());
        const double gta = d.mean();

        point3_adjust *= gta;

        // Get the transformation by aligning the points.
        Eigen::Matrix3d rmat;
        Eigen::Vector3d tvec;
        GetRigidTransform(point3_norm, point3_adjust, false, rmat, tvec);

        tvec = point3_var * tvec - rmat * point3_mean;
        focal_lengths[i] *= point2_var;

        ProjectionMatrix transformation_matrix;
        transformation_matrix.block<3, 3>(0, 0) = rmat;
        transformation_matrix.col(3) = tvec;
        Eigen::Matrix3d camera_matrix = Eigen::DiagonalMatrix<double, 3>(
            focal_lengths[i], focal_lengths[i], 1.0);
        projection_matrices.push_back(camera_matrix * transformation_matrix);
    }

    return static_cast<int>(projection_matrices.size());
}

} // namespace thoht
