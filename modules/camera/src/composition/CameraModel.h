#pragma once

#include "CameraModelBase.h"

namespace thoht {

template <typename PROJECTION_T, typename SHUTTER_T, typename MASK_T>
class CameraModel : public CameraModelBase
{
public:
    typedef PROJECTION_T projection_t;
    // Not all projections will have a distortion...is this necessary? Maybe.
    // typedef typename projection_t::distortion_t distortion_t;
    typedef typename projection_t::keypoint_t keypoint_t;
    typedef SHUTTER_T shutter_t;
    typedef MASK_T mask_t;
    typedef Frame<CameraModel<projection_t, shutter_t, mask_t>> frame_t;

    enum
    {
        KeypointDimension = projection_t::KeypointDimension
    };

    typedef Eigen::Matrix<double, KeypointDimension, 4> jacobian_homogeneous_t;
    typedef Eigen::Matrix<double, KeypointDimension, 3> jacobian_t;
    typedef Eigen::Matrix<double, 3, KeypointDimension> inverse_jacobian_t;
    typedef Eigen::Matrix<double, 4, KeypointDimension>
        inverse_jacobian_homogeneous_t;
    typedef Eigen::Matrix<double, KeypointDimension, KeypointDimension>
        covariance_t;

    CameraModel();
    CameraModel(const projection_t &projection);
    CameraModel(const projection_t &projection, const shutter_t &shutter);
    CameraModel(const projection_t &projection, const shutter_t &shutter,
                const mask_t &mask);
    virtual ~CameraModel();

    /// properties

    //////////////////////////////////////////////////////////////
    // PROJECTION FUNCTIONS
    //////////////////////////////////////////////////////////////
    template <typename DERIVED_P, typename DERIVED_K>
    bool euclideanToKeypoint(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const;

    template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
    bool euclideanToKeypoint(const Eigen::MatrixBase<DERIVED_P> &p,
                             const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
                             const Eigen::MatrixBase<DERIVED_JP> &outJp) const;

    template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
    bool euclideanToKeypointFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
        const Eigen::MatrixBase<DERIVED_JP> &outJp) const;

    template <typename DERIVED_P, typename DERIVED_K>
    bool homogeneousToKeypoint(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const;

    template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
    bool homogeneousToKeypoint(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
        const Eigen::MatrixBase<DERIVED_JP> &outJp) const;

    template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
    bool homogeneousToKeypointFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
        const Eigen::MatrixBase<DERIVED_JP> &outJp) const;

    // Project the point and get the associated uncertainty of the projection.
    template <typename DERIVED_K>
    bool homogeneousToKeypoint(
        const sm::kinematics::UncertainHomogeneousPoint &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
        covariance_t &outProjectionUncertainty) const;

    // Project the point
    template <typename DERIVED_K>
    bool homogeneousToKeypoint(
        const sm::kinematics::HomogeneousPoint &p,
        const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const;

    template <typename DERIVED_K, typename DERIVED_P>
    bool keypointToEuclidean(
        const Eigen::MatrixBase<DERIVED_K> &keypoint,
        const Eigen::MatrixBase<DERIVED_P> &outPoint) const;

    template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
    bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> &keypoint,
                             const Eigen::MatrixBase<DERIVED_P> &outPoint,
                             const Eigen::MatrixBase<DERIVED_JK> &outJk) const;

    template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
    bool keypointToEuclideanFiniteDifference(
        const Eigen::MatrixBase<DERIVED_K> &keypoint,
        const Eigen::MatrixBase<DERIVED_P> &outPoint,
        const Eigen::MatrixBase<DERIVED_JK> &outJk) const;

    template <typename DERIVED_K, typename DERIVED_P>
    bool keypointToHomogeneous(
        const Eigen::MatrixBase<DERIVED_K> &keypoint,
        const Eigen::MatrixBase<DERIVED_P> &outPoint) const;

    template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
    bool keypointToHomogeneous(
        const Eigen::MatrixBase<DERIVED_K> &keypoint,
        const Eigen::MatrixBase<DERIVED_P> &outPoint,
        const Eigen::MatrixBase<DERIVED_JK> &outJk) const;

    template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
    bool keypointToHomogeneousFiniteDifference(
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
    void euclideanToKeypointIntrinsicsJacobianFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JI> &outJi) const;

    template <typename DERIVED_P, typename DERIVED_JD>
    void euclideanToKeypointDistortionJacobianFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JD> &outJd) const;

    template <typename DERIVED_P, typename DERIVED_JI>
    void homogeneousToKeypointIntrinsicsJacobian(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JI> &outJi) const;

    template <typename DERIVED_P, typename DERIVED_JI>
    void homogeneousToKeypointIntrinsicsJacobianFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JI> &outJi) const;

    template <typename DERIVED_P, typename DERIVED_JS>
    void homogeneousToKeypointDistortionJacobian(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JS> &outJs) const;

    template <typename DERIVED_P, typename DERIVED_JD>
    void homogeneousToKeypointDistortionJacobianFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JD> &outJd) const;

    //////////////////////////////////////////////////////////////
    // SHUTTER SUPPORT
    //////////////////////////////////////////////////////////////

    // The amount of time elapsed between the start of the image and the
    // keypoint. For a global shutter camera, this can return Duration(0).
    template <typename DERIVED_K>
    Duration temporalOffset(const Eigen::MatrixBase<DERIVED_K> &keypoint) const;

    virtual Duration temporalOffset(const Eigen::VectorXd &keypoint) const;

    //////////////////////////////////////////////////////////////
    // VALIDITY TESTING
    //////////////////////////////////////////////////////////////

    template <typename Keypoint_t>
    bool isValid(const Eigen::MatrixBase<Keypoint_t> &keypoint) const;

    template <typename Point_t>
    bool isEuclideanVisible(const Eigen::MatrixBase<Point_t> &point) const;

    template <typename HomoPoint_t>
    bool isHomogeneousVisible(const Eigen::MatrixBase<HomoPoint_t> &ph) const;

    //////////////////////////////////////////////////////////////
    // SUPERCLASS SUPPORT
    //////////////////////////////////////////////////////////////
    bool vsEuclideanToKeypoint(const Eigen::Vector3d &p,
                               Eigen::VectorXd &outKeypoint) const override;

    bool vsEuclideanToKeypoint(const Eigen::Vector3d &p,
                               Eigen::VectorXd &outKeypoint,
                               Eigen::MatrixXd &outJacobian) const override;

    bool vsHomogeneousToKeypoint(const Eigen::Vector4d &ph,
                                 Eigen::VectorXd &outKeypoint) const override;

    bool vsHomogeneousToKeypoint(const Eigen::Vector4d &ph,
                                 Eigen::VectorXd &outKeypoint,
                                 Eigen::MatrixXd &outJacobian) const override;

    bool vsKeypointToEuclidean(const Eigen::VectorXd &keypoint,
                               Eigen::Vector3d &outPoint) const override;

    bool vsKeypointToEuclidean(const Eigen::VectorXd &keypoint,
                               Eigen::VectorXd &outPoint,
                               Eigen::MatrixXd &outJacobian) const override;

    bool vsKeypointToHomogeneous(Eigen::VectorXd const &keypoint,
                                 Eigen::VectorXd &outPoint) const override;

    bool vsKeypointToHomogeneous(Eigen::VectorXd const &keypoint,
                                 Eigen::VectorXd &outPoint,
                                 Eigen::MatrixXd &outJacobian) const override;

    bool vsIsValid(const Eigen::VectorXd &keypoint) const;

    bool vsIsEuclideanVisible(const Eigen::Vector3d &p) const;

    bool vsIsHomogeneousVisible(const Eigen::Vector4d &ph) const;

    // The amount of time elapsed between the start of the image and the
    // keypoint. For a global shutter camera, this can return Duration(0).
    virtual Duration vsTemporalOffset(const Eigen::VectorXd &keypoint) const;

    /// \brief the length of a keypoint
    virtual size_t keypointDimension() const;

    /// \brief is this an invertible camera model
    virtual bool isProjectionInvertible() const;

    //////////////////////////////////////////////////////////////
    // CAMERA CALIBRATION
    //////////////////////////////////////////////////////////////
    /// \brief initialize the intrinsics based on a list of views of a gridded
    /// calibration target \return true on success
    bool initializeIntrinsics(
        const std::vector<CalibObservation> &observations);

    /// \brief estimate the transformation of the camera with respect to the
    /// calibration target
    ///        On success out_T_t_c is filled in with the transformation that
    ///        takes points from the camera frame to the target frame
    /// \return true on success
    virtual bool estimateTransformation(const CalibObservation &obs,
                                        Eigen::Isometry3d &out_T_t_c) const;

    ////////////////////////////////////////////////////////////////
    // OPTIMIZATION SUPPORT
    ////////////////////////////////////////////////////////////////
    /// \brief the minimal dimensions of the projection parameters
    virtual int minimalDimensionsProjection() const;

    /// \brief the minimal dimensions of the distortion parameters
    virtual int minimalDimensionsDistortion() const;

    /// \brief the minimal dimensions of the shutter parameters
    virtual int minimalDimensionsShutter() const;

    // aslam::backend compatibility

    /// \brief update the intrinsics
    virtual void update(const double *v, bool projection, bool distortion,
                        bool shutter);

    /// \brief Get the total number of dimensions of the intrinsic parameters
    virtual int minimalDimensions(bool projection, bool distortion,
                                  bool shutter) const;

    /// \brief get the intrinsic parameters.
    virtual void getParameters(Eigen::MatrixXd &P, bool projection,
                               bool distortion, bool shutter) const;

    /// \brief set the intrinsic parameters.
    virtual void setParameters(const Eigen::MatrixXd &P, bool projection,
                               bool distortion, bool shutter);

    /// \brief return the Jacobian of the projection with respect to the
    /// intrinsics.
    virtual void euclideanToKeypointIntrinsicsJacobian(
        const Eigen::Vector3d &p, Eigen::MatrixXd &outJi,
        bool estimateProjection, bool estimateDistortion,
        bool estimateShutter) const;

    /// \brief return the Jacobian of the projection with respect to the
    /// intrinsics.
    virtual void homogeneousToKeypointIntrinsicsJacobian(
        const Eigen::Vector4d &p, Eigen::MatrixXd &outJi,
        bool estimateProjection, bool estimateDistortion,
        bool estimateShutter) const;

    /// \brief return the temporal offset with respect to the intrinsics.
    virtual void temporalOffsetIntrinsicsJacobian(
        const Eigen::VectorXd &keypoint, Eigen::MatrixXd &outJi,
        bool estimateProjection, bool estimateDistortion,
        bool estimateShutter) const;

    //////////////////////////////////////////////////////////////
    // UNIT TEST SUPPORT
    //////////////////////////////////////////////////////////////
    // \brief creates a random valid keypoint.
    virtual Eigen::VectorXd createRandomKeypoint() const;

    // \brief creates a random visible point. Negative depth means random
    // between 0 and 100 meters.
    virtual Eigen::Vector3d createRandomVisiblePoint(double depth = -1.0) const;

    projection_t &projection() { return _projection; }
    const projection_t &projection() const { return _projection; }

    shutter_t &shutter() { return _shutter; }
    const shutter_t &shutter() const { return _shutter; }

    void setMask(const mask_t &mask) { _mask = mask; }
    mask_t &mask() { return _mask; }
    const mask_t &mask() const { return _mask; }

    virtual bool hasMask() const { return _mask.isSet(); }

    virtual void print(std::ostream &out);

    bool isBinaryEqual(
        const CameraModel<PROJECTION_T, SHUTTER_T, MASK_T> &rhs) const;

    /// \brief This will create a frame with the right type. WARNING: it will
    /// not
    ///        fill in the camera geometry in the frame. This must be done
    ///        manually outside the call.
    virtual std::shared_ptr<FrameBase> createUninitializedFrameBase() const;

    /// \brief This will create a frame with the right type. WARNING: it will
    /// not
    ///        fill in the camera geometry in the frame. This must be done
    ///        manually outside the call.
    std::shared_ptr<frame_t> createUninitializedFrame() const;

    /// \todo redo this. Somehow Stefan's stuff needs it.
    virtual int width() const { return _projection.ru(); }
    virtual int height() const { return _projection.rv(); }

private:
    projection_t _projection;
    shutter_t _shutter;
    mask_t _mask;
};

///------ Implementation
template <typename Projection_t, typename Shutter_t, typename Mask_t>
CameraModel<Projection_t, Shutter_t, Mask_t>::CameraModel()
{
}

template <typename P, typename S, typename M>
CameraModel<P, S, M>::CameraModel(const projection_t &projection)
    : _projection(projection)
{
}

/// \brief Construct from projection and shutter
template <typename P, typename S, typename M>
CameraModel<P, S, M>::CameraModel(const projection_t &projection,
                                  const shutter_t &shutter)
    : _projection(projection), _shutter(shutter)
{
}

/// \brief Construct from projection, shutter, and mask
template <typename P, typename S, typename M>
CameraModel<P, S, M>::CameraModel(const projection_t &projection,
                                  const shutter_t &shutter, const mask_t &mask)
    : _projection(projection), _shutter(shutter), _mask(mask)
{
}

template <typename P, typename S, typename M>
CameraModel<P, S, M>::~CameraModel()
{
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsEuclideanToKeypoint(
    const Eigen::Vector3d &p, Eigen::VectorXd &outKeypoint) const
{
    bool valid = _projection.euclideanToKeypoint(p, outKeypoint);
    return valid && _mask.isValid(outKeypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsEuclideanToKeypoint(
    const Eigen::Vector3d &p, Eigen::VectorXd &outKeypoint,
    Eigen::MatrixXd &outJacobian) const
{
    bool valid = _projection.euclideanToKeypoint(p, outKeypoint, outJacobian);
    return valid && _mask.isValid(outKeypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsHomogeneousToKeypoint(
    const Eigen::Vector4d &ph, Eigen::VectorXd &outKeypoint) const
{
    bool valid = _projection.homogeneousToKeypoint(ph, outKeypoint);
    return valid && _mask.isValid(outKeypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsHomogeneousToKeypoint(
    const Eigen::Vector4d &ph, Eigen::VectorXd &outKeypoint,
    Eigen::MatrixXd &outJacobian) const
{
    bool valid =
        _projection.homogeneousToKeypoint(ph, outKeypoint, outJacobian);
    return valid && _mask.isValid(outKeypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsKeypointToEuclidean(
    const Eigen::VectorXd &keypoint, Eigen::Vector3d &outPoint) const
{
    return _projection.keypointToEuclidean(keypoint, outPoint) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsKeypointToEuclidean(
    const Eigen::VectorXd &keypoint, Eigen::VectorXd &outPoint,
    Eigen::MatrixXd &outJacobian) const
{
    return _projection.keypointToEuclidean(keypoint, outPoint, outJacobian) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsKeypointToHomogeneous(
    Eigen::VectorXd const &keypoint, Eigen::VectorXd &outPoint) const
{
    return _projection.keypointToHomogeneous(keypoint, outPoint) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsKeypointToHomogeneous(
    Eigen::VectorXd const &keypoint, Eigen::VectorXd &outPoint,
    Eigen::MatrixXd &outJacobian) const
{
    return _projection.keypointToHomogeneous(keypoint, outPoint, outJacobian) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsIsValid(const Eigen::VectorXd &keypoint) const
{
    return isValid(keypoint);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsIsEuclideanVisible(const Eigen::Vector3d &p) const
{
    return _projection.isEuclideanVisible(p);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::vsIsHomogeneousVisible(
    const Eigen::Vector4d &ph) const
{
    return _projection.isHomogeneousVisible(ph);
}

// The amount of time elapsed between the start of the image and the
// keypoint. For a global shutter camera, this can return Duration(0).
template <typename P, typename S, typename M>
Duration CameraModel<P, S, M>::vsTemporalOffset(
    const Eigen::VectorXd &keypoint) const
{
    return _shutter.temporalOffset(keypoint);
}

/// \brief get the camera id.
template <typename P, typename S, typename M>
CameraId CameraModel<P, S, M>::id() const
{
    return _id;
}

/// \brief set the camera id.
template <typename P, typename S, typename M>
void CameraModel<P, S, M>::setId(CameraId id)
{
    _id = id;
}

/// \brief the length of a keypoint
template <typename P, typename S, typename M>
size_t CameraModel<P, S, M>::keypointDimension() const
{
    return P::KeypointDimension;
}

// \brief creates a random valid keypoint.
template <typename P, typename S, typename M>
Eigen::VectorXd CameraModel<P, S, M>::createRandomKeypoint() const
{
    return _projection.createRandomKeypoint();
}

// \brief creates a random visible point. Negative depth means random between 0
// and 100 meters.
template <typename P, typename S, typename M>
Eigen::Vector3d CameraModel<P, S, M>::createRandomVisiblePoint(
    double depth) const
{
    return _projection.createRandomVisiblePoint(depth);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_K>
bool CameraModel<P, S, M>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const
{
    bool valid = _projection.euclideanToKeypoint(p, outKeypoint);
    return valid && _mask.isValid(outKeypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool CameraModel<P, S, M>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
    const Eigen::MatrixBase<DERIVED_JP> &outJp) const
{
    bool valid = _projection.euclideanToKeypoint(p, outKeypoint, outJp);
    return valid && _mask.isValid(outKeypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool CameraModel<P, S, M>::euclideanToKeypointFiniteDifference(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
    const Eigen::MatrixBase<DERIVED_JP> &outJp) const
{
    DERIVED_JP &Jp =
        const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp).derived();
    ASLAM_CAMERAS_ESTIMATE_JACOBIAN(this, euclideanToKeypoint, p, 1e-5, Jp);

    bool valid = _projection.euclideanToKeypoint(p, outKeypoint);
    return valid && _mask.isValid(outKeypoint);
}

// Project the point and get the associated uncertainty of the projection.
template <typename P, typename S, typename M>
template <typename DERIVED_K>
bool CameraModel<P, S, M>::homogeneousToKeypoint(
    const sm::kinematics::UncertainHomogeneousPoint &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
    covariance_t &outProjectionUncertainty) const
{
    jacobian_homogeneous_t J;
    bool r =
        static_cast<const CameraModel<projection_t, shutter_t, mask_t> *>(this)
            ->homogeneousToKeypoint(p.toHomogeneous(), outKeypoint, J);

    outProjectionUncertainty = J * p.U4() * J.transpose();
    return r;
}

template <typename P, typename S, typename M>
template <typename DERIVED_K>
bool CameraModel<P, S, M>::homogeneousToKeypoint(
    const sm::kinematics::HomogeneousPoint &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const
{
    return static_cast<const CameraModel<projection_t, shutter_t, mask_t> *>(
               this)
        ->homogeneousToKeypoint(p.toHomogeneous(), outKeypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_K>
bool CameraModel<P, S, M>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint) const
{
    bool valid = _projection.homogeneousToKeypoint(p, outKeypoint);
    return valid && _mask.isValid(outKeypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool CameraModel<P, S, M>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
    const Eigen::MatrixBase<DERIVED_JP> &outJp) const
{
    bool valid = _projection.homogeneousToKeypoint(p, outKeypoint, outJp);
    return valid && _mask.isValid(outKeypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool CameraModel<P, S, M>::homogeneousToKeypointFiniteDifference(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_K> &outKeypoint,
    const Eigen::MatrixBase<DERIVED_JP> &outJp) const
{
    DERIVED_JP &Jp =
        const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp).derived();
    ASLAM_CAMERAS_ESTIMATE_JACOBIAN(this, homogeneousToKeypoint, p, 1e-5, Jp);

    return _mask.isValid(_projection.homogeneousToKeypoint(p, outKeypoint));
}

template <typename P, typename S, typename M>
template <typename DERIVED_K, typename DERIVED_P>
bool CameraModel<P, S, M>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> &keypoint,
    const Eigen::MatrixBase<DERIVED_P> &outPoint) const
{
    // I'm putting the mask second in this case to make sure the
    // keypointToEuclidean function is called.
    return _projection.keypointToEuclidean(keypoint, outPoint) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool CameraModel<P, S, M>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> &keypoint,
    const Eigen::MatrixBase<DERIVED_P> &outPoint,
    const Eigen::MatrixBase<DERIVED_JK> &outJk) const
{
    // I'm putting the mask second in this case to make sure the
    // keypointToEuclidean function is called.
    return _projection.keypointToEuclidean(keypoint, outPoint, outJk) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool CameraModel<P, S, M>::keypointToEuclideanFiniteDifference(
    const Eigen::MatrixBase<DERIVED_K> &keypoint,
    const Eigen::MatrixBase<DERIVED_P> &outPoint,
    const Eigen::MatrixBase<DERIVED_JK> &outJk) const
{
    DERIVED_JK &Jk =
        const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk).derived();
    ASLAM_CAMERAS_ESTIMATE_JACOBIAN(this, keypointToEuclidean, keypoint, 1e-5,
                                    Jk);

    // I'm putting the mask second in this case to make sure the
    // keypointToEuclidean function is called.
    return _projection.keypointToEuclidean(keypoint, outPoint) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_K, typename DERIVED_P>
bool CameraModel<P, S, M>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> &keypoint,
    const Eigen::MatrixBase<DERIVED_P> &outPoint) const
{
    // I'm putting the mask second in this case to make sure the
    // keypointToEuclidean function is called.
    return _projection.keypointToHomogeneous(keypoint, outPoint) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool CameraModel<P, S, M>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> &keypoint,
    const Eigen::MatrixBase<DERIVED_P> &outPoint,
    const Eigen::MatrixBase<DERIVED_JK> &outJk) const
{
    // I'm putting the mask second in this case to make sure the
    // keypointToEuclidean function is called.
    return _projection.keypointToHomogeneous(keypoint, outPoint, outJk) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool CameraModel<P, S, M>::keypointToHomogeneousFiniteDifference(
    const Eigen::MatrixBase<DERIVED_K> &keypoint,
    const Eigen::MatrixBase<DERIVED_P> &outPoint,
    const Eigen::MatrixBase<DERIVED_JK> &outJk) const
{
    DERIVED_JK &Jk =
        const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk).derived();
    ASLAM_CAMERAS_ESTIMATE_JACOBIAN(this, keypointToHomogeneous, keypoint, 1e-5,
                                    Jk);

    // I'm putting the mask second in this case to make sure the
    // keypointToEuclidean function is called.
    return _projection.keypointToHomogeneous(keypoint, outPoint) &&
           _mask.isValid(keypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_JI>
void CameraModel<P, S, M>::euclideanToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_JI> &outJi) const
{
    _projection.euclideanToKeypointIntrinsicsJacobian(p, outJi);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_JD>
void CameraModel<P, S, M>::euclideanToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_JD> &outJd) const
{
    _projection.euclideanToKeypointDistortionJacobian(p, outJd);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_JI>
void CameraModel<P, S, M>::homogeneousToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_JI> &outJi) const
{
    _projection.homogeneousToKeypointIntrinsicsJacobian(p, outJi);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_JD>
void CameraModel<P, S, M>::homogeneousToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> &p,
    const Eigen::MatrixBase<DERIVED_JD> &outJd) const
{
    return _projection.homogeneousToKeypointDistortionJacobian(p, outJd);
}

// template<typename P, typename S, typename M>
// template<typename DERIVED_P, typename DERIVED_JD>
// void CameraModel<P, S, M>::homogeneousToKeypointShutterJacobian(
//     const Eigen::MatrixBase<DERIVED_P> & p,
//     const Eigen::MatrixBase<DERIVED_JD> & outJd) const {
//   return _shutter.homogeneousToKeypointShutterJacobian(p, outJd);
// }

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_JI>
void CameraModel<P, S, M>::
    euclideanToKeypointIntrinsicsJacobianFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JI> &outJi) const
{
    DERIVED_JI &Ji =
        const_cast<Eigen::MatrixBase<DERIVED_JI> &>(outJi).derived();
    projection_t proj = _projection;
    ASLAM_CAMERAS_ESTIMATE_INTRINSIC_JACOBIAN(euclideanToKeypoint, proj, proj,
                                              p, 1e-5, Ji);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_JI>
void CameraModel<P, S, M>::
    homogeneousToKeypointIntrinsicsJacobianFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JI> &outJi) const
{
    DERIVED_JI &Ji =
        const_cast<Eigen::MatrixBase<DERIVED_JI> &>(outJi).derived();
    projection_t proj = _projection;
    ASLAM_CAMERAS_ESTIMATE_INTRINSIC_JACOBIAN(homogeneousToKeypoint, proj, proj,
                                              p, 1e-5, Ji);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_JD>
void CameraModel<P, S, M>::
    euclideanToKeypointDistortionJacobianFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JD> &outJd) const
{
    DERIVED_JD &Jd =
        const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd).derived();
    projection_t proj = _projection;
    ASLAM_CAMERAS_ESTIMATE_INTRINSIC_JACOBIAN(euclideanToKeypoint, proj,
                                              proj.distortion(), p, 1e-5, Jd);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P, typename DERIVED_JD>
void CameraModel<P, S, M>::
    homogeneousToKeypointDistortionJacobianFiniteDifference(
        const Eigen::MatrixBase<DERIVED_P> &p,
        const Eigen::MatrixBase<DERIVED_JD> &outJd) const
{
    DERIVED_JD &Jd =
        const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd).derived();
    projection_t proj = _projection;
    ASLAM_CAMERAS_ESTIMATE_INTRINSIC_JACOBIAN(homogeneousToKeypoint, proj,
                                              proj.distortion(), p, 1e-5, Jd);
}

//////////////////////////////////////////////////////////////
// SHUTTER SUPPORT
//////////////////////////////////////////////////////////////

// The amount of time elapsed between the start of the image and the
// keypoint. For a global shutter camera, this can return
// CameraModel<P,S,M>::Duration(0).
template <typename P, typename S, typename M>
template <typename DERIVED_K>
Duration CameraModel<P, S, M>::temporalOffset(
    const Eigen::MatrixBase<DERIVED_K> &keypoint) const
{
    return _shutter.temporalOffset(keypoint);
}

template <typename P, typename S, typename M>
Duration CameraModel<P, S, M>::temporalOffset(
    const Eigen::VectorXd &keypoint) const
{
    return _shutter.temporalOffset(keypoint);
}

//////////////////////////////////////////////////////////////
// VALIDITY TESTING
//////////////////////////////////////////////////////////////

template <typename P, typename S, typename M>
template <typename DERIVED_K>
bool CameraModel<P, S, M>::isValid(
    const Eigen::MatrixBase<DERIVED_K> &keypoint) const
{
    return _mask.isValid(keypoint) && _projection.isValid(keypoint);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P>
bool CameraModel<P, S, M>::isEuclideanVisible(
    const Eigen::MatrixBase<DERIVED_P> &p) const
{
    keypoint_t k;
    bool success = _projection.euclideanToKeypoint(p, k);

    return success && _mask.isValid(k);
}

template <typename P, typename S, typename M>
template <typename DERIVED_P>
bool CameraModel<P, S, M>::isHomogeneousVisible(
    const Eigen::MatrixBase<DERIVED_P> &ph) const
{
    keypoint_t k;
    bool success = _projection.homogeneousToKeypoint(ph, k);

    return success && _mask.isValid(k);
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::isProjectionInvertible() const
{
    return _projection.isProjectionInvertible();
}

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::isBinaryEqual(const CameraModel<P, S, M> &rhs) const
{
    return SM_CHECKMEMBERSSAME(rhs, _id) &&
           SM_CHECKMEMBERSSAME(rhs, _projection) &&
           SM_CHECKMEMBERSSAME(rhs, _shutter) &&
           SM_CHECKMEMBERSSAME(rhs, _mask);
}

template <typename P, typename S, typename M>
CameraModel<P, S, M> CameraModel<P, S, M>::getTestGeometry()
{
    CameraModel<P, S, M> camera(projection_t::getTestProjection(),
                                shutter_t::getTestShutter(), mask_t());
    camera.setId(CameraId(rand()));
    return camera;
}

/// \brief initialize the intrinsics based on list of views of a gridded
/// calibration target

template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::initializeIntrinsics(
    const std::vector<GridCalibrationTargetObservation> &observations)
{
    return _projection.initializeIntrinsics(observations);
}

/// \brief estimate the transformation of the camera with respect to the
/// calibration target
template <typename P, typename S, typename M>
bool CameraModel<P, S, M>::estimateTransformation(
    const CalibObservation &obs, Eigen::Isometry3d &out_T_t_c) const
{
    return _projection.estimateTransformation(obs, out_T_t_c);
}

template <typename P, typename S, typename M>
std::shared_ptr<typename CameraModel<P, S, M>::frame_t>
CameraModel<P, S, M>::createUninitializedFrame() const
{
    std::shared_ptr<frame_t> frame(new frame_t);
    return frame;
}

template <typename P, typename S, typename M>
std::shared_ptr<aslam::FrameBase>
CameraModel<P, S, M>::createUninitializedFrameBase() const
{
    std::shared_ptr<FrameBase> frame(new frame_t);
    return frame;
}

template <typename PP, typename S, typename M>
void CameraModel<PP, S, M>::print(std::ostream &out)
{
    std::cout << "Printing!\n";
    Eigen::MatrixXd P;
    _projection.getParameters(P);
    out << "Projection: " << P.transpose() << std::endl;
    _projection.distortion().getParameters(P);
    if (P.rows() > 0 && P.cols() > 0)
    {
        out << "Distortion: " << P.transpose() << std::endl;
    }
}

/// \brief the minimal dimensions of the projection parameters
template <typename P, typename S, typename M>
int CameraModel<P, S, M>::minimalDimensionsProjection() const
{
    return _projection.minimalDimensions();
}

/// \brief the minimal dimensions of the distortion parameters
template <typename P, typename S, typename M>
int CameraModel<P, S, M>::minimalDimensionsDistortion() const
{
    return _projection.distortion().minimalDimensions();
}

template <typename P, typename S, typename M>
int CameraModel<P, S, M>::minimalDimensionsShutter() const
{
    return _shutter.minimalDimensions();
}

/// \brief update the intrinsics
template <typename P, typename S, typename M>
void CameraModel<P, S, M>::update(const double *v, bool estimateProjection,
                                  bool estimateDistortion, bool estimateShutter)
{
    if (estimateProjection)
    {
        _projection.update(v);
        v += _projection.minimalDimensions();
    }

    if (estimateDistortion)
    {
        _projection.distortion().update(v);
        v += _projection.distortion().minimalDimensions();
    }

    if (estimateShutter)
    {
        _shutter.update(v);
    }
}

/// \brief Get the total number of dimensions of the intrinsic parameters
template <typename P, typename S, typename M>
int CameraModel<P, S, M>::minimalDimensions(bool estimateProjection,
                                            bool estimateDistortion,
                                            bool estimateShutter) const
{
    int dim = 0;
    if (estimateProjection)
    {
        dim += _projection.minimalDimensions();
    }

    if (estimateDistortion)
    {
        dim += _projection.distortion().minimalDimensions();
    }

    if (estimateShutter)
    {
        dim += _shutter.minimalDimensions();
    }
    return dim;
}

/// \brief get the intrinsic parameters.
template <typename P, typename S, typename M>
void CameraModel<P, S, M>::getParameters(Eigen::MatrixXd &params,
                                         bool estimateProjection,
                                         bool estimateDistortion,
                                         bool estimateShutter) const
{
    Eigen::MatrixXd Pp, Pd, Ps;
    int rows = 0, cols = 0;
    if (estimateProjection)
    {
        _projection.getParameters(Pp);
        rows += Pp.rows();
        cols = std::max(cols, (int)Pp.cols());
    }

    if (estimateDistortion)
    {
        _projection.distortion().getParameters(Pd);
        rows += Pd.rows();
        cols = std::max(cols, (int)Pd.cols());
    }

    if (estimateShutter)
    {
        _shutter.getParameters(Ps);
        rows += Ps.rows();
        cols = std::max(cols, (int)Ps.cols());
    }

    params = Eigen::MatrixXd::Zero(rows, cols);
    int row = 0;

    if (estimateProjection)
    {
        params.block(row, 0, Pp.rows(), Pp.cols()) = Pp;
        row += Pp.rows();
    }

    if (estimateDistortion)
    {
        params.block(row, 0, Pd.rows(), Pd.cols()) = Pd;
        row += Pd.rows();
    }

    if (estimateShutter)
    {
        params.block(row, 0, Ps.rows(), Ps.cols()) = Ps;
        row += Ps.rows();
    }
}

/// \brief set the intrinsic parameters.
template <typename P, typename S, typename M>
void CameraModel<P, S, M>::setParameters(const Eigen::MatrixXd &params,
                                         bool estimateProjection,
                                         bool estimateDistortion,
                                         bool estimateShutter)
{
    // Ugh...I wish I had used a vector, not a matrix.
    int row = 0;
    if (estimateProjection)
    {
        Eigen::Vector2i sz = _projection.parameterSize();
        _projection.setParameters(params.block(row, 0, sz[0], sz[1]));
        row += sz[0];
    }

    if (estimateDistortion)
    {
        Eigen::Vector2i sz = _projection.distortion().parameterSize();
        _projection.distortion().setParameters(
            params.block(row, 0, sz[0], sz[1]));
        row += sz[0];
    }

    if (estimateShutter)
    {
        Eigen::Vector2i sz = _shutter.parameterSize();
        _shutter.setParameters(params.block(row, 0, sz[0], sz[1]));
        row += sz[0];
    }
}

/// \brief return the Jacobian of the projection with respect to the intrinsics.
template <typename P, typename S, typename M>
void CameraModel<P, S, M>::euclideanToKeypointIntrinsicsJacobian(
    const Eigen::Vector3d &p, Eigen::MatrixXd &outJi, bool estimateProjection,
    bool estimateDistortion, bool estimateShutter) const
{
    // Here I have to stack the answers. The resulting matrix should be
    // KeypointDimension x minimalDimensions()
    outJi = Eigen::MatrixXd::Zero(
        KeypointDimension,
        minimalDimensions(estimateProjection, estimateDistortion,
                          estimateShutter));

    int col = 0;
    if (estimateProjection)
    {
        Eigen::MatrixXd J;
        _projection.euclideanToKeypointIntrinsicsJacobian(p, J);
        outJi.block(col, 0, KeypointDimension,
                    _projection.minimalDimensions()) = J;
        col += _projection.minimalDimensions();
    }

    if (estimateDistortion)
    {
        Eigen::MatrixXd J;
        _projection.euclideanToKeypointDistortionJacobian(p, J);
        outJi.block(col, 0, KeypointDimension,
                    _projection.distortion().minimalDimensions()) = J;
        col += _projection.distortion().minimalDimensions();
    }

    if (estimateShutter)
    {
        // Uh...nothing. The shutter acts on the keypoint time.
        // At least for any example I can think of.
    }
}

/// \brief return the Jacobian of the projection with respect to the intrinsics.
template <typename P, typename S, typename M>
void CameraModel<P, S, M>::homogeneousToKeypointIntrinsicsJacobian(
    const Eigen::Vector4d &p, Eigen::MatrixXd &outJi, bool estimateProjection,
    bool estimateDistortion, bool estimateShutter) const
{
    // Here I have to stack the answers. The resulting matrix should be
    // KeypointDimension x minimalDimensions()
    outJi = Eigen::MatrixXd::Zero(
        KeypointDimension,
        minimalDimensions(estimateProjection, estimateDistortion,
                          estimateShutter));

    int col = 0;
    if (estimateProjection)
    {
        Eigen::MatrixXd J;
        _projection.homogeneousToKeypointIntrinsicsJacobian(p, J);
        outJi.block(0, col, KeypointDimension,
                    _projection.minimalDimensions()) = J;
        col += _projection.minimalDimensions();
    }

    if (estimateDistortion)
    {
        Eigen::MatrixXd J;
        _projection.homogeneousToKeypointDistortionJacobian(p, J);
        outJi.block(0, col, KeypointDimension,
                    _projection.distortion().minimalDimensions()) = J;
        col += _projection.distortion().minimalDimensions();
    }

    if (estimateShutter)
    {
        // Uh...nothing. The shutter acts on the keypoint time.
        // At least for any example I can think of.
    }
}

/// \brief return the temporal offset with respect to the intrinsics.
template <typename P, typename S, typename M>
void CameraModel<P, S, M>::temporalOffsetIntrinsicsJacobian(
    const Eigen::VectorXd &keypoint, Eigen::MatrixXd &outJi,
    bool estimateProjection, bool estimateDistortion,
    bool estimateShutter) const
{
    // Here I have to stack the answers. The resulting matrix should be
    // 1 x totalparameterSize
    outJi = Eigen::MatrixXd::Zero(
        1, minimalDimensions(estimateProjection, estimateDistortion,
                             estimateShutter));

    int col = 0;
    if (estimateProjection)
    {
        col += _projection.minimalDimensions();
    }

    if (estimateDistortion)
    {
        col += _projection.distortion().minimalDimensions();
    }

    if (estimateShutter)
    {
        _shutter.temporalOffsetIntrinsicsJacobian(
            keypoint, outJi.block(0, col, 1, _shutter.minimalDimensions()));
    }
}
} // namespace thoht
