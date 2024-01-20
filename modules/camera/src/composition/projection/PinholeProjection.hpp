#include "PinholeProjection.h"

#include "../../../core/util_eigen.h"

namespace tl {

template <typename Distortion_t>
PinholeProjection<Distortion_t>::PinholeProjection()
    : ProjectionBase<Distortion_t>()
{
    updateTemporaries();
}

template <typename Distortion_t>
PinholeProjection<Distortion_t>::PinholeProjection(double fu, double fv,
                                                   double cu, double cv, int ru,
                                                   int rv)
    : ProjectionBase<Distortion_t>(fu, fv, cu, cv, ru, rv)
{
    updateTemporaries();
}

template <typename Distortion_t>
PinholeProjection<Distortion_t>::PinholeProjection(double fu, double fv,
                                                   double cu, double cv, int ru,
                                                   int rv,
                                                   Distortion_t distortion)
    : ProjectionBase<Distortion_t>(fu, fv, cu, cv, ru, rv, distortion)
{
    updateTemporaries();
}

template <typename Distortion_t>
template <typename Point_t, typename Keypoint_t>
bool PinholeProjection<Distortion_t>::euclideanToKeypoint(
    const Eigen::MatrixBase<Point_t> &point,
    Eigen::MatrixBase<Keypoint_t> &keypoint) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Point_t>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Keypoint_t>, 2);

    const double rz = 1.0 / point[2];

    keypoint.derived().resize(2);
    keypoint[0] = point[0] * rz;
    keypoint[1] = point[1] * rz;

    this->_distortion.distort(keypoint);

    keypoint[0] = this->_fu * keypoint[0] + this->_cu;
    keypoint[1] = this->_fv * keypoint[1] + this->_cv;

    return isValid(keypoint) && point[2] > 0;
}

template <typename Distortion_t>
template <typename Point_t, typename Keypoint_t>
bool PinholeProjection<Distortion_t>::homogeneousToKeypoint(
    const Eigen::MatrixBase<Point_t> &ph,
    Eigen::MatrixBase<Keypoint_t> &outKeypoint) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Point_t>, 4);
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Keypoint_t>, 2);

    // hope this works... (required to have valid static asserts)
    if (ph[3] < 0)
        return euclideanToKeypoint(-ph.derived().template head<3>(),
                                   outKeypoint);

    return euclideanToKeypoint(ph.derived().template head<3>(), outKeypoint);
}

template <typename Distortion_t>
template <typename Keypoint_t, typename Point_t>
bool PinholeProjection<Distortion_t>::keypointToEuclidean(
    const Eigen::MatrixBase<Keypoint_t> &keypoint,
    Eigen::MatrixBase<Point_t> &point) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Point_t>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Keypoint_t>, 2);

    Keypoint_t kp = keypoint;

    kp[0] = (kp[0] - _cu) / _fu;
    kp[1] = (kp[1] - _cv) / _fv;
    _distortion.undistort(kp);

    point.derived().resize(3);

    point[0] = kp[0];
    point[1] = kp[1];
    point[2] = 1;

    return isValid(keypoint);
}

template <typename Distortion_t>
template <typename Keypoint_t, typename Point_t>
bool PinholeProjection<Distortion_t>::keypointToHomogeneous(
    const Eigen::MatrixBase<Keypoint_t> &keypoint,
    Eigen::MatrixBase<Point_t> &point) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Point_t>, 4);
    EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Keypoint_t>, 2);

    point.derived().resize(4);
    point[3] = 0.0;
    return keypointToEuclidean(keypoint, point.derived().template head<3>());
}

// template <typename Distortion_t>
// template <typename Point_t, typename Keypoint_t, typename Jacobian_t>
// bool PinholeProjection<Distortion_t>::euclideanToKeypoint(
//     const Eigen::MatrixBase<Point_t> &point,
//     Eigen::MatrixBase<Keypoint_t> &keypoint,
//     Eigen::MatrixBase<Jacobian_t> &J) const
//{
//     EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Point_t>, 3);
//     EIGEN_STATIC_ASSERT_VECTOR_SIZE(Eigen::MatrixBase<Keypoint_t>, 2);
//     EIGEN_STATIC_ASSERT_MATRIX_SIZE(Eigen::MatrixBase<Jacobian_t>, 2, 3);

//    const double rz = 1.0 / point[2];
//    const double rz2 = rz * rz;

//    keypoint.derived().resize(2);
//    J.derived().resize(KeypointDimension, 3);
//    J.setZero();

//    keypoint[0] = point[0] * rz;
//    keypoint[1] = point[1] * rz;

//    Eigen::MatrixXd Jd;
//    this->_distortion.distort(keypoint, Jd);

//    // Jacobian including distortion
//    J(0, 0) = this->_fu * Jd(0, 0) * rz;
//    J(0, 1) = this->_fu * Jd(0, 1) * rz;
//    J(0, 2) = -this->_fu * (point[0] * Jd(0, 0) + point[1] * Jd(0, 1)) * rz2;
//    J(1, 0) = this->_fv * Jd(1, 0) * rz;
//    J(1, 1) = this->_fv * Jd(1, 1) * rz;
//    J(1, 2) = -this->_fv * (point[0] * Jd(1, 0) + point[1] * Jd(1, 1)) * rz2;

//    keypoint[0] = this->_fu * keypoint[0] + this->_cu;
//    keypoint[1] = this->_fv * keypoint[1] + this->_cv;

//    return isValid(keypoint) && point[2] > 0;
//}

} // namespace tl
