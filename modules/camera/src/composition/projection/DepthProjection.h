#pragma once

#include <Eigen/Geometry>

#include "../../../core/util_eigen.h"
#include "ProjectionBase.h"

namespace thoht {

class CalibObservation;

template <typename Distortion_t>
class DepthProjection : public ProjectionBase<Distortion_t>
{
public:
    enum
    {
        KeypointDimension = 3
    };
    enum
    {
        IntrinsicsDimension = 4
    };
    enum
    {
        DesignVariableDimension = IntrinsicsDimension
    };

    using Keypoint = Eigen::Matrix<double, KeypointDimension, 1>;
    using JacobianIntrinsic =
        Eigen::Matrix<double, KeypointDimension, IntrinsicsDimension>;

    DepthProjection();
    explicit DepthProjection(Distortion_t distortion);
    DepthProjection(double focalLengthU, double focalLengthV,
                    double imageCenterU, double imageCenterV, int resolutionU,
                    int resolutionV, Distortion_t distortion);
    DepthProjection(double focalLengthU, double focalLengthV,
                    double imageCenterU, double imageCenterV, int resolutionU,
                    int resolutionV);
    virtual ~DepthProjection();

    // properties
    int keypointDim() const override { return KeypointDimension; }

    bool isProjectionInvertible() const override { return false; }

    // intrinsic
    // no extra params

    // point mapping
    template <typename DERIVED_P, typename DERIVED_K>
    bool euclideanToKeypoint(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const;

    template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
    bool euclideanToKeypoint(const Eigen::MatrixBase<DERIVED_P> &p,
                             const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
                             const Eigen::MatrixBase<DERIVED_JP> &outJp) const;

    template <typename DERIVED_K, typename DERIVED_P>
    bool keypointToEuclidean(
        const Eigen::MatrixBase<DERIVED_K> &keypoint,
        const Eigen::MatrixBase<DERIVED_P> &outPoint) const;

    template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
    bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> &keypoint,
                             const Eigen::MatrixBase<DERIVED_P> &outPoint,
                             const Eigen::MatrixBase<DERIVED_JK> &outJk) const;

    template <typename DERIVED_P, typename DERIVED_K>
    bool homogeneousToKeypoint(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const;

    template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
    bool homogeneousToKeypoint(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
        const Eigen::MatrixBase<DERIVED_JP> &outJp) const;

    template <typename DERIVED_K, typename DERIVED_P>
    bool keypointToHomogeneous(
        const Eigen::MatrixBase<DERIVED_K> &keypoint,
        const Eigen::MatrixBase<DERIVED_P> &outPoint) const;

    template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
    bool keypointToHomogeneous(
        const Eigen::MatrixBase<DERIVED_K> &keypoint,
        const Eigen::MatrixBase<DERIVED_P> &outPoint,
        const Eigen::MatrixBase<DERIVED_JK> &outJk) const;

    template <typename DERIVED_P, typename DERIVED_JI>
    void euclideanToKeypointIntrinsicsJacobian(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JI> &outJi) const;

    template <typename DERIVED_P, typename DERIVED_JD>
    void euclideanToKeypointDistortionJacobian(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JD> &outJd) const;

    template <typename DERIVED_P, typename DERIVED_JI>
    void homogeneousToKeypointIntrinsicsJacobian(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JI> &outJi) const;

    template <typename DERIVED_P, typename DERIVED_JD>
    void homogeneousToKeypointDistortionJacobian(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JD> &outJd) const;

    template <typename DERIVED_K>
    bool isValid(const Eigen::MatrixBase<DERIVED_K> &keypoint) const;

    template <typename DERIVED_P>
    bool isEuclideanVisible(const Eigen::MatrixBase<DERIVED_P> &p) const;

    template <typename DERIVED_P>
    bool isHomogeneousVisible(const Eigen::MatrixBase<DERIVED_P> &ph) const;

    /// \brief initialize the intrinsics based on one view of a gridded
    /// calibration target \return true on success
    bool initializeIntrinsics(
        const std::vector<CalibObservation> & /*observations*/)
    {
        //        SM_THROW(
        //            std::runtime_error,
        //            "initializeIntrinsics(): not implemented for
        //            DepthProjection!");
        return false;
    }

    /// \brief estimate the transformation of the camera with respect to the
    /// calibration target
    ///        On success out_T_t_c is filled in with the transformation that
    ///        takes points from the camera frame to the target frame
    /// \return true on success
    bool estimateTransformation(const CalibObservation & /* obs */,
                                Eigen::Isometry3d & /* out_T_t_c */) const
    {
        return false;
    }

    // aslam::backend compatibility
    void update(const double *v);
    int minimalDimensions() const;

    void getParameters(Eigen::MatrixXd &P) const;
    void setParameters(const Eigen::MatrixXd &P);
    Eigen::Vector2i parameterSize() const;

    // \brief creates a random valid keypoint.
    virtual Eigen::VectorXd createRandomKeypoint() const;

    // \brief creates a random visible point. Negative depth means random
    // between 0 and 100 meters.
    virtual Eigen::Vector3d createRandomVisiblePoint(double depth = -1.0) const;

private:
    /// \brief A computed value for speeding up computation.
    double _recip_fu;
    double _recip_fv;
    double _fu_over_fv;
};

template <typename DISTORTION_T>
DepthProjection<DISTORTION_T>::DepthProjection()
    : ProjectionBase<DISTORTION_T>(400., 400., 320., 240., 640., 480.),
      _recip_fu(1.0 / _fu),
      _recip_fv(1.0 / _fv),
      _fu_over_fv(_fu / _fv)
{
}

template <typename DISTORTION_T>
DepthProjection<DISTORTION_T>::DepthProjection(DISTORTION_T distortion)
    : ProjectionBase<DISTORTION_T>(400., 400., 320., 240., 640., 480.,
                                   distortion),
      _recip_fu(1.0 / _fu),
      _recip_fv(1.0 / _fv),
      _fu_over_fv(_fu / _fv)
{
}

template <typename DISTORTION_T>
DepthProjection<DISTORTION_T>::DepthProjection(double fu, double fv, double cu,
                                               double cv, int ru, int rv,
                                               DISTORTION_T distortion)
    : ProjectionBase<DISTORTION_T>(fu, fv, cu, cv, ru, rv, distortion),
      _recip_fu(1.0 / _fu),
      _recip_fv(1.0 / _fv),
      _fu_over_fv(_fu / _fv)
{
}

template <typename DISTORTION_T>
DepthProjection<DISTORTION_T>::DepthProjection(double fu, double fv, double cu,
                                               double cv, int ru, int rv)
    : ProjectionBase<DISTORTION_T>(fu, fv, cu, cv, ru, rv),
      _recip_fu(1.0 / _fu),
      _recip_fv(1.0 / _fv),
      _fu_over_fv(_fu / _fv)
{
}

template <typename DISTORTION_T>
DepthProjection<DISTORTION_T>::~DepthProjection()
{
}

template <typename DISTORTION_T>
template <typename DERIVED_P, typename DERIVED_K>
bool DepthProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypointConst) const
{
    /// \todo
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 3);

    auto outKeypoint =
        const_cast<Eigen::MatrixBase<DERIVED_K> &>(outKeypointConst);

    outKeypoint.derived().resize(3);
    double rz = 1.0 / p[2];
    outKeypoint[0] = p[0] * rz;
    outKeypoint[1] = p[1] * rz;
    outKeypoint[2] = rz;

    //_distortion.distort(outKeypoint);

    outKeypoint[0] = _fu * outKeypoint[0] + _cu;
    outKeypoint[1] = _fv * outKeypoint[1] + _cv;

    return isValid(outKeypoint) && p[2] > 0;
}

template <typename DISTORTION_T>
template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool DepthProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypointConst,
    const Eigen::MatrixBase<DERIVED_JP> &outJp) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JP>, 3, 3);
    /// \todo
    Eigen::MatrixBase<DERIVED_K> &outKeypoint =
        const_cast<Eigen::MatrixBase<DERIVED_K> &>(outKeypointConst);
    outKeypoint.derived().resize(3);

    // Jacobian:
    Eigen::MatrixBase<DERIVED_JP> &J =
        const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
    J.derived().resize(KeypointDimension, 3);
    J.setZero();

    double rz = 1.0 / p[2];
    double rz2 = rz * rz;
    outKeypoint[0] = p[0] * rz;
    outKeypoint[1] = p[1] * rz;
    outKeypoint[2] = rz;

    Eigen::MatrixXd Jd;
    //_distortion.distort(outKeypoint, Jd);

    outKeypoint[0] = _fu * outKeypoint[0] + _cu;
    outKeypoint[1] = _fv * outKeypoint[1] + _cv;

    // Jacobian including distortion
    J(0, 0) = _fu * Jd(0, 0) * rz;
    J(0, 1) = _fu * Jd(0, 1) * rz;
    J(0, 2) = -_fu * (p[0] * Jd(0, 0) + p[1] * Jd(0, 1)) * rz2;
    J(1, 0) = _fv * Jd(1, 0) * rz;
    J(1, 1) = _fv * Jd(1, 1) * rz;
    J(1, 2) = -_fv * (p[0] * Jd(1, 0) + p[1] * Jd(1, 1)) * rz2;

    /// \todo does the distortion affect the depth?
    J(2, 2) = -rz2;

    outKeypoint[0] = _fu * outKeypoint[0] + _cu;
    outKeypoint[1] = _fv * outKeypoint[1] + _cv;

    return isValid(outKeypoint) && p[2] > 0;
}

template <typename DISTORTION_T>
template <typename DERIVED_P, typename DERIVED_K>
bool DepthProjection<DISTORTION_T>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> &ph,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const
{
    /// \todo this class now assumes (as the pinhole projection class) that a
    /// homogeneous point ph has p[3] == 0!
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 3);

    // hope this works... (required to have valid static asserts)
    if (ph[3] < 0)
        return euclideanToKeypoint(-ph.derived().template head<3>(),
                                   outKeypoint);
    else
        return euclideanToKeypoint(ph.derived().template head<3>(),
                                   outKeypoint);
}

template <typename DISTORTION_T>
template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool DepthProjection<DISTORTION_T>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> &ph,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
    const Eigen::MatrixBase<DERIVED_JP> &outJp) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JP>, 3, 4);

    Eigen::MatrixBase<DERIVED_JP> &J =
        const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
    J.derived().resize(KeypointDimension, 4);
    J.setZero();

    // hope this works... (required to have valid static asserts)
    return euclideanToKeypoint(
        ph.derived().template head<3>(), outKeypoint,
        J.derived().template topLeftCorner<KeypointDimension, 3>());

    if (ph[3] < 0)
        return euclideanToKeypoint(
            ph.derived().template head<3>(), outKeypoint,
            J.derived().template topLeftCorner<KeypointDimension, 3>());
    else
        return euclideanToKeypoint(
            ph.derived().template head<3>(), outKeypoint,
            J.derived().template topLeftCorner<KeypointDimension, 3>());
}

template <typename DISTORTION_T>
template <typename DERIVED_K, typename DERIVED_P>
bool DepthProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & /* keypoint */,
    const Eigen::MatrixBase<DERIVED_P> & /* outPointConst */) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 3);
    /// \todo
    // keypoint_t kp = keypoint;
    /*
     kp[0] = (kp[0] - _cu) / _fu;
     kp[1] = (kp[1] - _cv) / _fv;
     _distortion.undistort(kp); // revert distortion

    Eigen::MatrixBase<DERIVED_P> & outPoint =
    const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPointConst);
    outPoint.derived().resize(3);

     outPoint[0] = kp[0];
     outPoint[1] = kp[1];
     outPoint[2] = 1;
     */
    return false; // isValid(keypoint);
}

template <typename DISTORTION_T>
template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool DepthProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & /* keypoint */,
    const Eigen::MatrixBase<DERIVED_P> & /* outPointConst */,
    const Eigen::MatrixBase<DERIVED_JK> & /* outJk */) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JK>, 3, 3);
    /// \todo
    // keypoint_t kp = keypoint;
    /*
     kp[0] = (kp[0] - _cu) / _fu;
     kp[1] = (kp[1] - _cv) / _fv;

    Eigen::MatrixXd Jd(2, 2);

     _distortion.undistort(kp, Jd); // revert distortion

    Eigen::MatrixBase<DERIVED_P> & outPoint =
    const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPointConst);
    outPoint.derived().resize(3);

     outPoint[0] = kp[0];
     outPoint[1] = kp[1];
     outPoint[2] = 1;

    Eigen::MatrixBase<DERIVED_JK> & J = const_cast<Eigen::MatrixBase<DERIVED_JK>
    &>(outJk); J.derived().resize(3,KeypointDimension); J.setZero();

     //J.derived()(0,0) = 1.0;
     //J.derived()(1,1) = _fu_over_fv;

    J.derived()(0,0) = _recip_fu;
    J.derived()(1,1) = _recip_fv;

     J *= Jd;
     */
    return false; // isValid(keypoint);
}

template <typename DISTORTION_T>
template <typename DERIVED_K, typename DERIVED_P>
bool DepthProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> &keypoint,
    const Eigen::MatrixBase<DERIVED_P> &outPoint) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 3);

    Eigen::MatrixBase<DERIVED_P> &p =
        const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
    p.derived().resize(4);
    p[3] = 1.0;
    return keypointToEuclidean(keypoint, p.derived().template head<3>());
}

template <typename DISTORTION_T>
template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool DepthProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> &keypoint,
    const Eigen::MatrixBase<DERIVED_P> &outPoint,
    const Eigen::MatrixBase<DERIVED_JK> &outJk) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JK>, 4, 3);

    Eigen::MatrixBase<DERIVED_JK> &Jk =
        const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);
    Jk.derived().resize(KeypointDimension, 4);
    Jk.setZero();

    Eigen::MatrixBase<DERIVED_P> &p =
        const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
    p[3] = 1.0;

    return keypointToEuclidean(
        keypoint, p.template head<3>(),
        Jk.template topLeftCorner<3, KeypointDimension>());
}

template <typename DISTORTION_T>
template <typename DERIVED_P, typename DERIVED_JI>
void DepthProjection<DISTORTION_T>::euclideanToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & /* p */,
    const Eigen::MatrixBase<DERIVED_JI> &outJi) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JI>, 3, 4);
    /// \todo
    Eigen::MatrixBase<DERIVED_JI> &J =
        const_cast<Eigen::MatrixBase<DERIVED_JI> &>(outJi);
    J.derived().resize(KeypointDimension, 4);
    J.setZero();
    /*
     double rz = 1.0/p[2];

    keypoint_t kp;
    kp[0] = p[0] * rz;
    kp[1] = p[1] * rz;
    _distortion.distort(kp);

     J(0,0) = kp[0];
     J(0,2) = 1;

    J(1,1) = kp[1];
    J(1,3) = 1;*/
}

template <typename DISTORTION_T>
template <typename DERIVED_P, typename DERIVED_JD>
void DepthProjection<DISTORTION_T>::euclideanToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & /* p */,
    const Eigen::MatrixBase<DERIVED_JD> & /* outJd */) const
{
    /// \todo
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    /*
     double rz = 1.0/p[2];
     keypoint_t kp;
     kp[0] = p[0] * rz;
     kp[1] = p[1] * rz;

    _distortion.distortParameterJacobian(kp, outJd);

     Eigen::MatrixBase<DERIVED_JD> & J =
    const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);
     J.derived().resize(KeypointDimension, _distortion.minimalDimensions());

    J.row(0) *= _fu;
    J.row(1) *= _fv;*/
}

template <typename DISTORTION_T>
template <typename DERIVED_P, typename DERIVED_JI>
void DepthProjection<DISTORTION_T>::homogeneousToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & /* p */,
    const Eigen::MatrixBase<DERIVED_JI> & /* outJi */) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JI>, 3, 4);

    //    SM_THROW(std::runtime_error, "Not Implemented");
    // euclideanToKeypointIntrinsicsJacobian(p.derived().template
    // head<3>()(0,3), outJi);
}

template <typename DISTORTION_T>
template <typename DERIVED_P, typename DERIVED_JD>
void DepthProjection<DISTORTION_T>::homogeneousToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_JD> &outJd) const
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 4);

    euclideanToKeypointDistortionJacobian(p.derived().template head<3>(),
                                          outJd);
}

// \brief creates a random valid keypoint.
template <typename DISTORTION_T>
Eigen::VectorXd DepthProjection<DISTORTION_T>::createRandomKeypoint() const
{
    /// \todo
    return Eigen::VectorXd::Zero(KeypointDimension);
}

// \brief creates a random visible point. Negative depth means random between 0
// and 100 meters.
template <typename DISTORTION_T>
Eigen::Vector3d DepthProjection<DISTORTION_T>::createRandomVisiblePoint(
    double /* depth */) const
{
    /// \todo
    return Eigen::VectorXd::Zero(3);
}

template <typename DISTORTION_T>
template <typename DERIVED_K>
bool DepthProjection<DISTORTION_T>::isValid(
    const Eigen::MatrixBase<DERIVED_K> &keypoint) const
{
    /// \todo
    return keypoint[0] >= 0 && keypoint[1] >= 0 && keypoint[0] < (double)_ru &&
           keypoint[1] < (double)_rv;
}

template <typename DISTORTION_T>
template <typename DERIVED_P>
bool DepthProjection<DISTORTION_T>::isEuclideanVisible(
    const Eigen::MatrixBase<DERIVED_P> &p) const
{
    /// \todo
    keypoint_t k;
    return euclideanToKeypoint(p, k);
}

template <typename DISTORTION_T>
template <typename DERIVED_P>
bool DepthProjection<DISTORTION_T>::isHomogeneousVisible(
    const Eigen::MatrixBase<DERIVED_P> &ph) const
{
    /// \todo
    keypoint_t k;
    return homogeneousToKeypoint(ph, k);
}

template <typename DISTORTION_T>
void DepthProjection<DISTORTION_T>::update(const double *v)
{
    _fu += v[0];
    _fv += v[1];
    _cu += v[2];
    _cv += v[3];
    _recip_fu = 1.0 / _fu;
    _recip_fv = 1.0 / _fv;
    _fu_over_fv = _fu / _fv;
}

template <typename DISTORTION_T>
int DepthProjection<DISTORTION_T>::minimalDimensions() const
{
    return IntrinsicsDimension;
}

template <typename DISTORTION_T>
Eigen::Vector2i DepthProjection<DISTORTION_T>::parameterSize() const
{
    return Eigen::Vector2i(4, 1);
}

template <typename DISTORTION_T>
void DepthProjection<DISTORTION_T>::getParameters(Eigen::MatrixXd &P) const
{
    P.resize(4, 1);
    P(0, 0) = _fu;
    P(1, 0) = _fv;
    P(2, 0) = _cu;
    P(3, 0) = _cv;
}

template <typename DISTORTION_T>
void DepthProjection<DISTORTION_T>::setParameters(const Eigen::MatrixXd &P)
{
    _fu = P(0, 0);
    _fv = P(1, 0);
    _cu = P(2, 0);
    _cv = P(3, 0);
    _recip_fu = 1.0 / _fu;
    _recip_fv = 1.0 / _fv;
    _fu_over_fv = _fu / _fv;
}
} // namespace thoht
