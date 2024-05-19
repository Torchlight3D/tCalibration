#pragma once

#include <memory>

#include <tMath/Eigen/Types>
#include <tCamera/Types>

#include "camera_meta_data.h"

namespace tl {

// TODO:
// 1. Use CRTP to simplify Point Mapping. Try my TemplateFactory
class CameraIntrinsics
{
public:
    using Ptr = std::shared_ptr<CameraIntrinsics>;
    using ConstPtr = std::shared_ptr<const CameraIntrinsics>;

    enum class Type
    {
        Pinhole = 0,
        PinholeRadialTangential = 1,
        Fisheye = 2, // a.k.a. KannalaBrandt(KB), Equidistant
        Fov = 3,
        DivisionUndistortion = 4,
        DoubleSphere = 5,
        ExtendedUnified = 6,
        Orthographic = 7,
        Omnidirectional = 8, // a.k.a Mei
    };

    CameraIntrinsics() = default;
    virtual ~CameraIntrinsics() = default;

    virtual CameraIntrinsics& operator=(const CameraIntrinsics& other);

    // Factory method
    static Ptr create(Type type);

    virtual constexpr Type type() const = 0;

    virtual void setFromMetaData(const CameraMetaData& meta);
    virtual CameraMetaData toMetaData() const;

    /// ----------------------- Parameters Access ------------------------------
    ///
    enum IntrinsicsIndex
    {
        Fx = 0,
        YX,
        Cx,
        Cy,

        ExtraIndex, // Learn from Qt::UserRole
    };
    virtual int numParameters() const = 0;

    // NOTE: In case some camera models can't generate the focal length in
    // common sense.
    inline static constexpr auto kMagicFocalLength{314.};
    void setFocalLength(double fx);
    double focalLength() const;
    inline void setFocalLength(double fx, double fy)
    {
        setFocalLengthX(fx);
        setAspectRatio(fy / fx);
    }
    inline void setFocalLengthX(double fx) { setFocalLength(fx); }
    inline auto focalLengthX() const { return focalLength(); }
    inline auto focalLengthY() const { return focalLengthX() * aspectRatio(); }
    inline auto fx() const { return focalLengthX(); }
    inline auto fy() const { return focalLengthY(); }

    // Height / Width
    void setAspectRatio(double aspectRatio);
    double aspectRatio() const;

    void setPrincipalPoint(double cx, double cy);
    double principalPointX() const;
    double principalPointY() const;
    inline auto cx() const { return principalPointX(); }
    inline auto cy() const { return principalPointY(); }

    void setParameter(int index, double value);
    double parameter(int index) const;

    const double* parameters() const;
    double* rParameters();
    std::vector<double> vector() const;

    // Returns the indices of the parameters that will be optimized during BA
    virtual std::vector<int> constantParameterIndices(
        OptimizeIntrinsicsType flags) const;

    virtual Eigen::Matrix3d calibrationMatrix() const;
    inline auto matrixK() const { return calibrationMatrix(); }

    // It's impossible to know if the intrinsic is "valid" based on its values.
    // Here "valid" only means all the values are not default values, or exceed
    // empirical/experimental range.
    virtual bool isValid() const;

    /// ------------------------- Point Mapping ----------------------------
    ///
    // Projects the 3D point in the camera coordinate system into the image
    // plane and distorts the point according to the radial distortion
    // parameters.
    virtual Eigen::Vector2d spaceToImage(
        const Eigen::Vector3d& point) const = 0;

    // Converts image pixel coordinates to normalized coordinates in the camera
    // coordinates by removing the effect of camera intrinsics/calibration.
    virtual Eigen::Vector3d imageToSpace(
        const Eigen::Vector2d& pixel) const = 0;

    // Apply or remove radial distortion to the given point. Points should be
    // given in **normalized** coordinates such that the effects of camera
    // intrinsics are not present.
    virtual Eigen::Vector2d distort(
        const Eigen::Vector2d& undistorted) const = 0;
    virtual Eigen::Vector2d undistort(
        const Eigen::Vector2d& distorted) const = 0;

    // Use float to adapt OpenCV interface
    virtual void createUndistortionMap(Eigen::MatrixXf& map1,
                                       Eigen::MatrixXf& map2,
                                       const Eigen::Vector2i& imageSize,
                                       double scale = 1.);

    virtual void createUndistortionRectifyMap(
        Eigen::MatrixXf& map1, Eigen::MatrixXf& map2, Eigen::Matrix3d& newK,
        const Eigen::Vector2i& imageSize,
        const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity());

    /// Debug
    friend std::ostream& operator<<(std::ostream& os,
                                    const CameraIntrinsics& cam);

protected:
    virtual std::string toLog() const;

protected:
    std::vector<double> parameters_;
};

} // namespace tl
