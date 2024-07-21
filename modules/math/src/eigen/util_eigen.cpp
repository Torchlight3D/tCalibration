#include "util_eigen.h"

#include <Eigen/EigenValues>

#include <tCore/Math>

namespace tl::math {

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {

size_t findNearestTimestamp(double t, const std::vector<double>& ts,
                            double& distance)
{
    distance = kMaxDouble;
    size_t index{0};
    for (size_t i{0}; i < ts.size(); ++i) {
        double dist = std::abs(t - ts[i]);
        if (dist < distance) {
            distance = dist;
            index = i;

            if (isApprox0(distance)) {
                break;
            }
        }
    }

    return index;
}

} // namespace

Eigen::Vector3d Lerp(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1,
                     double fraction)
{
    return (1.0 - fraction) * v0 + fraction * v1;
}

void InterpolateVector3s(const std::vector<double>& ts_old,
                         const std::vector<double>& ts_new,
                         const Vector3dList& src, Vector3dList& interpolated)
{
    assert(src.size() == ts_old.size());

    for (const auto& t : ts_new) {
        double nearest_dist;
        auto nearest_idx = findNearestTimestamp(t, ts_old, nearest_dist);

        if (nearest_idx < ts_new.size()) {
            const double fraction =
                nearest_dist / (ts_old[nearest_idx + 1] - ts_old[nearest_idx]);
            interpolated.push_back(
                Lerp(src[nearest_idx], src[nearest_idx + 1], fraction));
        }
        else {
            interpolated.push_back(src[nearest_idx]);
        }
    }
}

void InterpolateQuaternions(const std::vector<double>& ts_old,
                            const std::vector<double>& ts_new,
                            const QuaterniondList& src,
                            QuaterniondList& interpolated)
{
    assert(src.size() == ts_old.size());

    for (const auto& t : ts_new) {
        double nearest_dist;
        auto nearest_idx = findNearestTimestamp(t, ts_old, nearest_dist);

        if (nearest_idx < ts_old.size() - 1) {
            const double fraction =
                nearest_dist / (ts_old[nearest_idx + 1] - ts_old[nearest_idx]);
            interpolated.push_back(
                src[nearest_idx].slerp(fraction, src[nearest_idx + 1]));
        }
        else {
            interpolated.push_back(src[nearest_idx]);
        }
    }
}

Eigen::MatrixXd MatrixSquareRoot(const Eigen::MatrixXd& mat)
{
    CHECK_EQ(mat.rows(), mat.cols());

    Eigen::EigenSolver<MatrixXd> solver{mat};
    const MatrixXd V = solver.eigenvectors().real();
    const VectorXd Dv = solver.eigenvalues().real();
    const MatrixXd sqrt_D = Dv.cwiseSqrt().asDiagonal();

    return V * sqrt_D * V.inverse();
}

Eigen::MatrixXd MatrixSquareRootForSemidefinitePositiveMat(
    const Eigen::MatrixXd& mat)
{
    CHECK_EQ(mat.rows(), mat.cols());

    Eigen::LDLT<Eigen::MatrixXd> ldlt{mat};

    Eigen::MatrixXd result;
    result = ldlt.matrixL();
    result = ldlt.transpositionsP().transpose() * result;
    result *= ldlt.vectorD().array().sqrt().matrix().asDiagonal();

    return result;
}

} // namespace tl::math
