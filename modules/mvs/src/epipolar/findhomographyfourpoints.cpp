#include "findhomographyfourpoints.h"

#include <Eigen/LU>
#include <Eigen/SVD>
#include <glog/logging.h>

#include "basics.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::RowVector3d;

using Matrix29d = Eigen::Matrix<double, 2, 9>;
using MatrixX9d = Eigen::Matrix<double, Eigen::Dynamic, 9>;
using Vector9d = Eigen::Matrix<double, 9, 1>;

namespace {

inline Matrix29d CreateActionConstraint(const Vector2d& img1_point,
                                        const Vector2d& img2_point)
{
    // [ 0    0  0  -x1  -y1  -1  x1y2   y1y2   y2]
    // [ x1  y1  1   0    0   0  -x1x2  -y1x2  -x2]
    Matrix29d constraint;
    constraint << RowVector3d::Zero(), -img1_point.transpose(), -1.,
        img1_point.transpose() * img2_point.y(), img2_point.y(),
        img1_point.transpose(), 1., RowVector3d::Zero(),
        -img1_point.transpose() * img2_point.x(), -img2_point.x();

    return constraint;
}

} // namespace

bool FourPointHomography(const std::vector<Eigen::Vector2d>& imgPoints1,
                         const std::vector<Eigen::Vector2d>& imgPoints2,
                         Eigen::Matrix3d* homography)
{
    CHECK_GE(imgPoints1.size(), 4);
    CHECK_EQ(imgPoints1.size(), imgPoints2.size());

    // Normalize the image points.
    Matrix3d norm_image_1_mat, norm_image_2_mat;
    const auto norm_image_1_points =
        NormalizeImagePoints(imgPoints1, &norm_image_1_mat);
    const auto norm_image_2_points =
        NormalizeImagePoints(imgPoints2, &norm_image_2_mat);

    // Create the constraint matrix based on x' = Hx (Eq. 4.1 in Hartley and
    // Zisserman).
    MatrixX9d action_matrix(2 * imgPoints1.size(), 9);
    for (int i = 0; i < imgPoints1.size(); i++) {
        action_matrix.block<2, 9>(2 * i, 0) = CreateActionConstraint(
            norm_image_1_points[i], norm_image_2_points[i]);
    }

    const Vector9d null_vector = (action_matrix.transpose() * action_matrix)
                                     .jacobiSvd(Eigen::ComputeFullV)
                                     .matrixV()
                                     .rightCols<1>();

    *homography = norm_image_2_mat.inverse() *
                  Eigen::Map<const Matrix3d>(null_vector.data()).transpose() *
                  norm_image_1_mat;
    return true;
}

bool FindHomographyFourPoint(const std::vector<Eigen::Vector2d>& imagePoints1,
                             const std::vector<Eigen::Vector2d>& imagePoints2,
                             Eigen::Matrix3d* homography)
{
    return FourPointHomography(imagePoints1, imagePoints2, homography);
}

} // namespace tl
