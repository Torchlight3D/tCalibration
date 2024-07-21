#pragma once

#include <numeric>

#include <ceres/rotation.h>
#include <Eigen/Geometry>

#include "eigen_types.h"

namespace tl {

namespace math {

template <typename T>
void OmegaToSkew(const Eigen::Vector3<T>& omega, Eigen::Matrix4<T>& skew)
{
    // clang-format off
    skew <<     T(0), -omega(0), -omega(1), -omega(2),
            omega(0),      T(0),  omega(2), -omega(1),
            omega(1), -omega(2),      T(0),  omega(0),
            omega(2),  omega(1), -omega(0),      T(0);
    // clang-format on
}

template <typename T>
Eigen::Matrix3<T> Skew(const Eigen::Vector3<T>& vec)
{
    // clang-format off
    return (Eigen::Matrix3<T>() <<   T(0), -vec(2),  vec(1),
                                   vec(2),    T(0), -vec(0),
                                  -vec(1),  vec(0),    T(0))
        .finished();
    // clang-format on
}

// Brief:
// Rotation conversion.
//
// Explanation:
// Euler(Roll-Pitch-Yaw, ZYX), RotationMatrix, AngleAxis, Quaternion
template <typename T>
Eigen::Quaternion<T> ExpQ(const Eigen::Quaternion<T>& quat)
{
    const T norm = quat.vec().norm();
    const T exp_w = std::exp(quat.w());
    if (norm == T(0)) {
        return {exp_w, T(0), T(0), T(0)};
    }

    Eigen::Quaternion<T> res;
    res.w() = exp_w * T(std::cos(norm));
    res.vec() = exp_w * T(sinc(norm)) * quat.vec();

    return res;
}

template <typename T>
Eigen::Quaternion<T> LogQ(const Eigen::Quaternion<T>& quat)
{
    T exp_w = quat.norm();
    T w = std::log(exp_w);
    T a = std::acos(quat.w() / exp_w);
    if (a == T(0)) {
        return {w, T(0), T(0), T(0)};
    }

    Eigen::Quaternion<T> res;
    res.w() = w;
    res.vec() = quat.vec() / exp_w / (sin(a) / a);

    return res;
}

// TODO: finish this, please
enum class RotationConvention
{
    YXZ_Local,
    ZXY_World,
    ZXZ_World
};

template <typename T>
void RotationMatrixToRPY(const Eigen::Matrix3<T>& rmat, T& roll, T& pitch,
                         T& yaw)
{
    roll = std::atan2(rmat(2, 1), rmat(2, 2));
    pitch = std::atan2(-rmat(2, 0), std::sqrt(rmat(2, 1) * rmat(2, 1) +
                                              rmat(2, 2) * rmat(2, 2)));
    yaw = std::atan2(rmat(1, 0), rmat(0, 0));
}

template <typename T>
Eigen::Vector3<T> RotationMatrixToAngleAxis(const Eigen::Matrix3<T>& rmat)
{
    Eigen::AngleAxis<T> aa;
    aa.fromRotationMatrix(rmat);
    return aa.angle() * aa.axis();
}

template <typename T>
void QuaternionToRPY(const Eigen::Quaternion<T>& q, T& roll, T& pitch, T& yaw)
{
    const T r11 = T(2) * (q.x() * q.y() + q.w() * q.z());
    const T r12 = q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z();
    const T r21 = -T(2) * (q.x() * q.z() - q.w() * q.y());
    const T r31 = T(2) * (q.y() * q.z() + q.w() * q.x());
    const T r32 = q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();
    roll = std::atan2(r11, r12);
    pitch = std::asin(r21);
    yaw = std::atan2(r31, r32);
}

template <typename T>
inline void QuaternionToRPY(const Eigen::Quaternion<T>& q,
                            Eigen::Vector3<T>& rpy)
{
    QuaternionToRPY(q, rpy[0], rpy[1], rpy[2]);
}

template <typename T>
Eigen::Matrix3<T> QuaternionToRotationMatrix(const T* const q)
{
    T R[9];
    ceres::QuaternionToRotation(q, R);

    // TODO: use cleaner way
    Eigen::Matrix3<T> rmat;
    for (int i{0}; i < 3; ++i) {
        for (int j{0}; j < 3; ++j) {
            rmat(i, j) = R[i * 3 + j];
        }
    }

    return rmat;
}

template <typename T>
void QuaternionToAngleAxis(const Eigen::Quaternion<T>& quat,
                           Eigen::Vector3<T>& rvec)
{
    // Quaternion<T> -> T*: q.coeffs().data()
    Eigen::Matrix3<T> rmat = quat.toRotationMatrix();

    Eigen::AngleAxis<T> aa;
    aa.fromRotationMatrix(rmat);
    rvec = aa.angle() * aa.axis();
}

// WARNING: input quaternion is in [x, y, z, w]
template <typename T>
inline void QuaternionToAngleAxis(const T* const quat, Eigen::Vector3<T>& rvec)
{
    // [x, y, z, w] -> [w, x, y, z]
    QuaternionToAngleAxis(
        Eigen::Quaternion<T>{quat[3], quat[0], quat[1], quat[2]}, rvec);
}

template <typename T>
Eigen::Matrix3<T> AngleAxisToRotationMatrix(const Eigen::Vector3<T>& angleAxis)
{
    const T angle = angleAxis.norm();
    // TODO: use isApprox0
    if (angle == T(0)) {
        return Eigen::Matrix3<T>::Identity();
    }

    const Eigen::Vector3<T> axis = angleAxis.normalized();
    return Eigen::Matrix3<T>{Eigen::AngleAxis<T>(angle, axis)};
}

template <typename T>
inline Eigen::Quaternion<T> AngleAxisToQuaternion(const Eigen::Vector3<T>& rvec)
{
    const auto rmat = AngleAxisToRotationMatrix(rvec);
    return Eigen::Quaternion<T>{rmat};
}

template <typename T>
void AngleAxisToQuaternion(const Eigen::Vector3<T>& rvec, T* quat)
{
    const auto q = AngleAxisToQuaternion(rvec);
    quat[0] = q.x();
    quat[1] = q.y();
    quat[2] = q.z();
    quat[3] = q.w();
}

// NOTE: We also can use
// 1. Eigen::Map, and inline; 2. Eigen::AngleAxis.
// Compare their result later.
template <typename T>
void AngleAxisToQuaternion(const T* const rvec, T* quat)
{
    const T& a0 = rvec[0];
    const T& a1 = rvec[1];
    const T& a2 = rvec[2];
    const T theta_squared = a0 * a0 + a1 * a1 + a2 * a2;

    // When points not at the origin, the full conversion is numerically stable.
    if (theta_squared > T(0.0)) {
        const T theta = std::sqrt(theta_squared);
        const T half_theta = theta * T(0.5);
        const T k = sin(half_theta) / theta;
        quat[0] = cos(half_theta);
        quat[1] = a0 * k;
        quat[2] = a1 * k;
        quat[3] = a2 * k;
    }
    else {
        // At the origin, sqrt() will produce NaN in the derivative since
        // the argument is zero.  By approximating with a Taylor series,
        // and truncating at one term, the value and first derivatives will be
        // computed correctly when Jets are used.
        const T k(0.5);
        quat[0] = T(1.0);
        quat[1] = a0 * k;
        quat[2] = a1 * k;
        quat[3] = a2 * k;
    }
}

template <typename T>
Eigen::Quaternion<T> AverageQuaternion(
    const std::vector<Eigen::Quaternion<T>>& quats)
{
    using namespace Eigen;

    Matrix3<T> sum =
        std::accumulate(quats.begin(), quats.end(), Matrix3<T>::Zero(),
                        [](Matrix3<T>& sum, const Quaternion<T>& quat) {
                            return sum + quat.toRotationMatrix();
                        });

    JacobiSVD<Matrix3<T>> svd{sum, ComputeFullU | ComputeFullV};
    const auto& U = svd.matrixU();
    const auto& V = svd.matrixV();

    Matrix3<T> result = U *
                        Eigen::DiagonalMatrix<T, 3>(
                            T(1), T(1), U.determinant() * V.determinant()) *
                        V.transpose();

    return Quaternion<T>{result};
}

// TODO: be careful about the order
template <typename T>
void NormalizeQuaternion(Eigen::Vector4<T>& quat)
{
    quat /= quat.norm();
}

template <typename T>
inline void NormalizeQuaternion(T quat[4])
{
    auto tmp = Eigen::Map<Eigen::Vector4<T>>(quat);
    NormalizeQuaternion(tmp);
}

template <typename T>
Eigen::Matrix4<T> QuaternionMultMatLeft(const Eigen::Quaternion<T>& q)
{
    // clang-format off
    return (Eigen::Matrix4<T>() << q.w(), -q.z(),  q.y(), q.x(),
                                   q.z(),  q.w(), -q.x(), q.y(),
                                  -q.y(),  q.x(),  q.w(), q.z(),
                                  -q.x(), -q.y(), -q.z(), q.w())
        .finished();
    // clang-format on
}

template <typename T>
Eigen::Matrix4<T> QuaternionMultMatRight(const Eigen::Quaternion<T>& q)
{
    // clang-format off
    return (Eigen::Matrix4<T>() << q.w(),  q.z(), -q.y(), q.x(),
                                  -q.z(),  q.w(),  q.x(), q.y(),
                                   q.y(), -q.x(),  q.w(), q.z(),
                                  -q.x(), -q.y(), -q.z(), q.w())
        .finished();
    // clang-format on
}

// TODO: It's weird to show up here
inline void transformPoint(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                           double scale, Eigen::Vector3d& point)
{
    point = scale * R * point + t;
}

Eigen::Vector3d Lerp(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1,
                     double fraction);

void InterpolateVector3s(const std::vector<double>& timestampsOld,
                         const std::vector<double>& timestampsNew,
                         const Vector3dList& source,
                         Vector3dList& interpolated);

void InterpolateQuaternions(const std::vector<double>& timestampsOld,
                            const std::vector<double>& timestampsNew,
                            const QuaterniondList& source,
                            QuaterniondList& interpolated);

// Brief:
// Perform a 4th order Runge-Kutta integration
//
// Input:
//     quat: The input Eigen 4D vector representing the initial rotation
//     omega0: Initial rotational velocity at time t0
//     omega1: Final rotational velocity at time t1
//     dt: Time step (t1 - t0).
// Output:
//     out_quat: Resulting final rotation

template <typename T>
void QuaternionIntegrationByRK4(const Eigen::Vector4<T>& quat,
                                const Eigen::Vector3<T>& omega0,
                                const Eigen::Vector3<T>& omega1, double dt,
                                Eigen::Vector4<T>& quat_out)
{
    using namespace Eigen;

    Matrix4<T> omega_skew;
    // 1st Runge-Kutta coefficient
    OmegaToSkew(omega0, omega_skew);
    const Vector4<T> k1 = T(0.5) * omega_skew * quat;

    Vector4<T> tmp_q;
    // 2nd Runge-Kutta coefficient
    tmp_q = quat + T(0.5) * dt * k1;
    const Vector3<T> omega01 = T(0.5) * (omega0 + omega1);
    OmegaToSkew(omega01, omega_skew);
    const Vector4<T> k2 = T(0.5) * omega_skew * tmp_q;

    // 3rd Runge-Kutta coefficient (same omega skew as second coeff.)
    tmp_q = quat + T(0.5) * dt * k2;
    const Vector4<T> k3 = T(0.5) * omega_skew * tmp_q;

    // 4th Runge-Kutta coefficient
    tmp_q = quat + dt * k3;
    OmegaToSkew(omega1, omega_skew);
    const Vector4<T> k4 = T(0.5) * omega_skew * tmp_q;
    using namespace Eigen;
    T mult1 = T(1.0) / T(6.0), mult2 = T(1.0) / T(3.0);
    quat_out = quat + dt * (mult1 * k1 + mult2 * k2 + mult2 * k3 + mult1 * k4);
    NormalizeQuaternion(quat_out);
}

template <typename T>
inline void QuaternionIntegrationByRK4(const T quat[4], const T omega0[3],
                                       const T omega1[3], double dt,
                                       T quat_out[4])
{
    using namespace Eigen;

    const auto q = Map<const Vector4<T>>(quat);
    const auto w0 = Map<const Vector3<T>>(omega0);
    const auto w1 = Map<const Vector3<T>>(omega1);

    Vector4<T> res_quat;
    QuaternionIntegrationByRK4(q, w0, w1, dt, res_quat);

    quat_out[0] = res_quat(0);
    quat_out[1] = res_quat(1);
    quat_out[2] = res_quat(2);
    quat_out[3] = res_quat(3);
}

template <typename T>
void QuaternionIntegrationByMidPoint(const Eigen::Quaternion<T>& quat,
                                     const Eigen::Vector3<T>& omega0,
                                     const Eigen::Vector3<T>& omega1, double dt,
                                     Eigen::Quaternion<T>& quat_out)
{
    const auto mid = (omega0 + omega1) * 0.5;
    const auto delta = mid * dt * 0.5;
    quat_out = quat * Eigen::Quaternion<T>{T(1), delta(0), delta(1), delta(2)};
}

inline bool ArraysEqualUpToScale(int n, const double* p, const double* q,
                                 double tolerance)
{
    Eigen::Map<const Eigen::VectorXd> p_vec(p, n);
    Eigen::Map<const Eigen::VectorXd> q_vec(q, n);

    // Use the cos term in the dot product to determine equality normalized for
    // scale.
    const double cos_diff = p_vec.dot(q_vec) / (p_vec.norm() * q_vec.norm());
    return std::abs(cos_diff) >= 1.0 - tolerance;
}

// Implements the RQ decomposition that recovers matrices R and Q such that
// A = R * Q where R is an mxn upper-triangular matrix and Q is an nxn unitary
// matrix.
template <typename MatrixType>
class RQDecomposition
{
public:
    using MatrixTransposeType = Eigen::Matrix<
        typename MatrixType::Scalar, MatrixType::ColsAtCompileTime,
        MatrixType::RowsAtCompileTime,
        (MatrixType::Flags & Eigen::RowMajorBit) ? Eigen::RowMajor
                                                 : Eigen::ColMajor,
        MatrixType::MaxColsAtCompileTime, MatrixType::MaxRowsAtCompileTime>;

    using MatrixQType =
        typename Eigen::HouseholderQR<MatrixTransposeType>::MatrixQType;

    explicit RQDecomposition(const MatrixType& matrix);

    // Mimic Eigen's QR interface.
    const MatrixType& matrixR() const { return R_; }

    const MatrixQType& matrixQ() const { return Q_; }

private:
    MatrixType R_;
    MatrixQType Q_;
};

/// ------------------ RQDecomposition Implementation ---------------------

// The matlab version of RQ decomposition is as follows:
//
// function [R Q] = rq(M)
//   [Q,R] = qr(flipud(M)')
//   R = flipud(R');
//   R = fliplr(R);
//   Q = Q';
//   Q = flipud(Q);
//
// where flipup flips the matrix upside-down and fliplr flips the matrix
// from left to right.
template <typename MatrixType>
RQDecomposition<MatrixType>::RQDecomposition(const MatrixType& matrix)
{
    // flipud(M)' = fliplr(M').
    const MatrixTransposeType matrix_flipud_transpose =
        matrix.transpose().rowwise().reverse();

    Eigen::HouseholderQR<MatrixTransposeType> qr(matrix_flipud_transpose);
    const MatrixQType& q0 = qr.householderQ();
    const MatrixTransposeType& r0 = qr.matrixQR();

    // Flip upside down.
    R_ = r0.transpose();
    R_ = R_.colwise().reverse().eval();

    // Flip left right.
    R_ = R_.rowwise().reverse().eval();

    // When R is an mxn matrix and m <= n then it is upper triangular.
    // If m > n then all elements below the subdiagonal are 0.
    for (int r{0}; r < R_.rows(); ++r) {
        for (int c{0}; R_.cols() - c > R_.rows() - r && c < R_.cols(); ++c) {
            R_(r, c) = 0;
        }
    }

    // Flip upside down.
    Q_ = q0.transpose();
    Q_ = Q_.colwise().reverse().eval();

    // Since the RQ decomposition is not unique, we will simply require that
    // det(Q) = 1. This makes using RQ decomposition for decomposing the
    // projection matrix very simple.
    if (Q_.determinant() < 0) {
        Q_.row(1) *= -1.0;
        R_.col(1) *= -1.0;
    }
}

Eigen::MatrixXd MatrixSquareRoot(const Eigen::MatrixXd& mat);

Eigen::MatrixXd MatrixSquareRootForSemidefinitePositiveMat(
    const Eigen::MatrixXd& mat);

} // namespace math

} // namespace tl

// A generic hash function that hashes constant size Eigen matrices and vectors.
template <typename Scalar_t, int Row_t, int Col_t>
struct std::hash<Eigen::Matrix<Scalar_t, Row_t, Col_t>>
{
    size_t operator()(const Eigen::Matrix<Scalar_t, Row_t, Col_t>& matrix) const
    {
        const auto* data = matrix.data();
        const hash<Scalar_t> hasher{};

        size_t seed{0};
        for (Eigen::Index i{0}; i < matrix.size(); ++i) {
            seed ^= hasher(data[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
