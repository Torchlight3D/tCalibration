#include "p4p_focal_distortion.h"
#include "impl/p4p_focal_distortion_impl.h"

#include <glog/logging.h>

#include <tMath/RandomGenerator>

namespace tl {

namespace {

inline double sgn(double val) { return (0.0 < val) - (val < 0.0); }

} // namespace

using Matrix34d = Eigen::Matrix<double, 3, 4>;
using Matrix24d = Eigen::Matrix<double, 2, 4>;
using Matrix42d = Eigen::Matrix<double, 4, 2>;
using Vector8d = Eigen::Matrix<double, 8, 1>;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

bool P4PFocalDistortionOptions::isValid() const
{
    return min_focal_length >= 0. && max_focal_length >= min_focal_length &&
           min_distortion <= 0. && max_distortion <= min_distortion;
}

bool P4PFocalDistortionOptions::isValidFocalLength(double f) const
{
    return !(f < min_focal_length || f > max_focal_length);
}

bool P4PFocalDistortionOptions::isValidDistortion(double d) const
{
    return !(d < max_distortion || d > min_distortion);
}

bool FourPointsPoseFocalLengthRadialDistortion(
    const Vector2dList& image_points, const Vector3dList& world_points,
    const P4PFocalDistortionOptions& opts, Matrix3dList* rotations,
    Vector3dList* translations, std::vector<double>* radial_distortions,
    std::vector<double>* focal_lengths)
{
    // check that input size of features and world points is 4
    CHECK_EQ(image_points.size(), world_points.size());
    CHECK(opts.isValid());

    using Matrix34d = Eigen::Matrix<double, 3, 4>;
    using Matrix24d = Eigen::Matrix<double, 2, 4>;
    using Matrix42d = Eigen::Matrix<double, 4, 2>;
    using Vector8d = Eigen::Matrix<double, 8, 1>;
    using Vector5d = Eigen::Matrix<double, 5, 1>; // distortion for this model

    Vector4d d;
    Matrix34d point3;
    Matrix34d point2;
    for (int i = 0; i < 4; ++i) {
        d[i] = image_points[i].squaredNorm();
        point2.col(i) = image_points[i].homogeneous();
        point3.col(i) = world_points[i];
    }

    const Vector3d point3_mean = point3.rowwise().mean();

    Matrix<double, 4, 4, Eigen::DontAlign> U;
    U.topRows<3>() = point3.colwise() - point3_mean;
    U.bottomRows<1>() << 1.0, 1.0, 1.0, 1.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd{
        U.topRows<3>(), Eigen::ComputeFullV | Eigen::ComputeFullU};

    Matrix3d R0 = svd.matrixU();
    if (sgn(R0.determinant()) < 0.0) {
        R0.col(0) *= -1;
    }
    R0.transposeInPlace();

    U.topRows<3>() = R0 * U.topRows<3>();
    const double scale =
        U.topRows<3>().array().pow(2).colwise().sum().sqrt().mean();

    U.topRows<3>() /= scale;

    // rescale image points
    const double f0 =
        point2.topRows<2>().array().pow(2).colwise().sum().sqrt().mean();
    point2.topRows<2>() /= f0;

    const double k0 = d.array().mean();
    d /= k0;

    Matrix<double, 5, 8> M;
    M.fill(0.0);
    M.row(0).leftCols<4>() = U.col(0);
    M.row(1).rightCols<4>() = U.col(0);
    for (int k = 1; k < 4; ++k) {
        M.row(k + 1).leftCols<4>() =
            point2.row(1).col(k) * U.col(k).transpose();
        M.row(k + 1).rightCols<4>() =
            -point2.row(0).col(k) * U.col(k).transpose();
    }

    Vector5d b;
    b.fill(0.0);
    b.topRows<2>() = point2.col(0).topRows<2>();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr;
    qr.compute(M.transpose());

    Matrix<double, 8, 8> Q = qr.householderQ();
    Matrix<double, 8, 5> R = qr.matrixQR().triangularView<Eigen::Upper>();
    Matrix<double, 8, 4> N;
    N.fill(0.0);
    N.leftCols<3>() = Q.rightCols<3>();

    // Use random rotation to make the solver more stable
    static RandomNumberGenerator rng(42);
    Vector3d rvec(rng.RandDouble(-0.5, 0.5), rng.RandDouble(-0.5, 0.5),
                  rng.RandDouble(-0.5, 0.5));
    Eigen::AngleAxisd random_rot(rvec.norm(), rvec);
    N.leftCols<3>() *= random_rot.toRotationMatrix();
    Matrix<double, 8, 1> x0 =
        Q.leftCols<5>() * (R.topRows<5>().transpose().fullPivLu().solve(b));
    N.rightCols<1>() = x0;

    Matrix<double, 6, 3> C;
    C.fill(0.0);
    Matrix34d UN1 = U.rightCols<3>().transpose() * N.topRows<4>();
    Matrix34d UN2 = U.rightCols<3>().transpose() * N.bottomRows<4>();
    Matrix<double, 6, 9> B;
    B.fill(0.0);

    B.topLeftCorner<3, 3>() = UN1.leftCols<3>();
    B.bottomLeftCorner<3, 3>() = UN2.leftCols<3>();
    B.topRightCorner<3, 1>() = UN1.rightCols<1>();
    B.bottomRightCorner<3, 1>() = UN2.rightCols<1>();

    B.block<3, 4>(0, 3) =
        d.bottomRows<3>().transpose().replicate(4, 1).transpose().cwiseProduct(
            UN1);
    B.block<3, 4>(3, 3) =
        d.bottomRows<3>().transpose().replicate(4, 1).transpose().cwiseProduct(
            UN2);

    B.col(7).topRows<3>() =
        -point2.row(0).rightCols<3>().transpose().cwiseProduct(
            U.row(2).rightCols<3>().transpose());
    B.col(7).bottomRows<3>() =
        -point2.row(1).rightCols<3>().transpose().cwiseProduct(
            U.row(2).rightCols<3>().transpose());

    // fill these up
    Matrix3d Utmp;
    Utmp.row(0) = U.row(0).rightCols<3>();
    Utmp.row(1) = U.row(1).rightCols<3>();
    Utmp.row(2) = U.row(3).rightCols<3>();

    Matrix3d u1temp;
    u1temp.row(0) = point2.row(0).rightCols<3>();
    u1temp.row(1) = point2.row(0).rightCols<3>();
    u1temp.row(2) = point2.row(0).rightCols<3>();
    Matrix3d u2temp;
    u2temp.row(0) = point2.row(1).rightCols<3>();
    u2temp.row(1) = point2.row(1).rightCols<3>();
    u2temp.row(2) = point2.row(1).rightCols<3>();

    C.block<3, 3>(0, 0) = Utmp.transpose().cwiseProduct(u1temp.transpose());
    C.block<3, 3>(3, 0) = Utmp.transpose().cwiseProduct(u2temp.transpose());

    Matrix<double, 3, 9> D = C.colPivHouseholderQr().solve(B);
    Map<Eigen::RowVectorXd> N_(N.data(), N.size());
    Map<Eigen::RowVectorXd> D_(D.data(), D.size());

    Matrix<double, 64, 1> data;
    data(0) = 0.0; // used to keep matlab indices, just an index offset of 1
    data.block<32, 1>(1, 0) = N_;
    data.block<27, 1>(33, 0) = D_;
    data(60, 0) = d(0, 0);
    data.bottomRows<3>() = U.topRows<3>().col(0);

    std::vector<Vector5d> valid_solutions;
    FourPointsPoseFocalLengthRadialDistortionSolver(data, &valid_solutions);

    for (const auto& sol : valid_solutions) {
        const double k = sol[3];
        const double P33 = sol[4];
        Eigen::Vector4d alpha(sol[0], sol[1], sol[2], 1.);

        Vector8d P12_ = N * alpha;
        Map<Matrix42d> P12(P12_.data(), 4, 2);

        Matrix<double, 9, 1> tmp;
        tmp(0, 0) = alpha[0];
        tmp(1, 0) = alpha[1];
        tmp(2, 0) = alpha[2];
        tmp(3, 0) = k * alpha(0);
        tmp(4, 0) = k * alpha(1);
        tmp(5, 0) = k * alpha(2);
        tmp(6, 0) = k;
        tmp(7, 0) = P33;
        tmp(8, 0) = 1.0;

        const Vector3d P3_124 = D * tmp;

        Vector4d P3;
        P3(0) = P3_124(0);
        P3(1) = P3_124(1);
        P3(2) = P33;
        P3(3) = P3_124(2);

        Matrix34d P;
        P.topRows<2>() = P12.transpose();
        P.bottomRows<1>() = P3;
        P /= P.bottomRows<1>().leftCols<3>().norm();
        const double f = P.topRows<1>().leftCols<3>().norm();
        Matrix3d K = Matrix3d::Identity();
        K(0, 0) = 1. / f;
        K(1, 1) = 1. / f;

        const double focal_length_estimate = f * f0;
        if (!opts.isValidFocalLength(focal_length_estimate)) {
            continue;
        }

        const double radial_distortion_estimate = k / k0;
        if (!opts.isValidDistortion(radial_distortion_estimate)) {
            continue;
        }

        focal_lengths->emplace_back(focal_length_estimate);
        radial_distortions->emplace_back(radial_distortion_estimate);

        Matrix34d Rt = K * P;

        if (Rt.topLeftCorner<3, 3>().determinant() < 0.0) {
            Rt *= -1.0;
        }

        // scale radial distortion with focal length
        //(*radial_distortions)[i] *= ((*focal_lengths)[i]*(*focal_lengths)[i]);
        Rt.col(3) =
            Rt.col(3) * scale - Rt.topLeftCorner<3, 3>() * R0 * point3_mean;
        Rt.topLeftCorner<3, 3>() = Rt.topLeftCorner<3, 3>() * R0;

        rotations->emplace_back(Rt.topLeftCorner<3, 3>());
        translations->emplace_back(Rt.col(3));
    }

    return !valid_solutions.empty();
}

} // namespace tl
