#include "findfundamentaleightpoints.h"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <glog/logging.h>

#include "basics.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

using MatrixX9d = Eigen::Matrix<double, Eigen::Dynamic, 9>;
using Vector9d = Eigen::Matrix<double, 9, 1>;

bool FindFundamentalEightPoints(const std::vector<Vector2d>& imgPoints1,
                                const std::vector<Vector2d>& imgPoints2,
                                Matrix3d* fmatrix)
{
    CHECK_EQ(imgPoints1.size(), imgPoints2.size());
    CHECK_GE(imgPoints1.size(), 8);

    // Normalize the image points.
    Matrix3d img1_norm_mat, img2_norm_mat;
    const auto norm_img1_points =
        NormalizeImagePoints(imgPoints1, &img1_norm_mat);
    const auto norm_img2_points =
        NormalizeImagePoints(imgPoints2, &img2_norm_mat);

    // Build the constraint matrix based on x2' * F * x1 = 0.
    MatrixX9d constraint_matrix(imgPoints1.size(), 9);
    for (size_t i{0}; i < imgPoints1.size(); i++) {
        constraint_matrix.block<1, 3>(i, 0) = norm_img1_points[i].homogeneous();
        constraint_matrix.block<1, 3>(i, 0) *= norm_img2_points[i].x();
        constraint_matrix.block<1, 3>(i, 3) = norm_img1_points[i].homogeneous();
        constraint_matrix.block<1, 3>(i, 3) *= norm_img2_points[i].y();
        constraint_matrix.block<1, 3>(i, 6) = norm_img1_points[i].homogeneous();
    }

    // Solve the constraint equation for F from nullspace extraction.
    // An LU decomposition is efficient for the minimally constrained case.
    // Otherwise, use an SVD.
    Vector9d normalized_fvector;
    if (imgPoints1.size() == 8) {
        const auto lu_decomposition = constraint_matrix.fullPivLu();
        if (lu_decomposition.dimensionOfKernel() != 1) {
            return false;
        }
        normalized_fvector = lu_decomposition.kernel();
    }
    else {
        Eigen::JacobiSVD<MatrixX9d> cmatrix_svd(constraint_matrix,
                                                Eigen::ComputeFullV);
        normalized_fvector = cmatrix_svd.matrixV().col(8);
    }

    // NOTE: This is the transpose of a valid fundamental matrix! We implement a
    // "lazy" transpose and defer it to the SVD a few lines below.
    Eigen::Map<const Matrix3d> normalized_fmatrix(normalized_fvector.data());

    // Find the closest singular matrix to F under frobenius norm. We can
    // compute this matrix with SVD.
    Eigen::JacobiSVD<Matrix3d> svd{normalized_fmatrix.transpose(),
                                   Eigen::ComputeFullU | Eigen::ComputeFullV};
    auto singularValues = svd.singularValues();
    singularValues[2] = 0.;
    *fmatrix =
        svd.matrixU() * singularValues.asDiagonal() * svd.matrixV().transpose();

    // Correct for the point normalization.
    *fmatrix = img2_norm_mat.transpose() * (*fmatrix) * img1_norm_mat;

    return true;
}

bool NormalizedEightPointFundamentalMatrix(
    const std::vector<Eigen::Vector2d>& imagePoints1,
    const std::vector<Eigen::Vector2d>& imagePoints2, Eigen::Matrix3d* fmatrix)
{
    return FindFundamentalEightPoints(imagePoints1, imagePoints2, fmatrix);
}

} // namespace tl
