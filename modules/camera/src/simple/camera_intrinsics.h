#pragma once

#include <memory>

#include <AxCore/EnumUtils>
#include <AxMath/EigenTypes>

#include "camera_meta_data.h"

namespace thoht {

// Explanation:
// The camera intrinsics parameters are defined by:
//   - Focal length
//   - Aspect ratio (y/x)
//   - Principal points (x, y)
//   - Skew
//   - Radial distortion
//   - Tangential distortion
// These intrinsic parameters may or may not be present for a given camera model
// and only the relevant intrinsics will be optimized per camera.
// It's common to assume that skew is 0, or aspect ratio is 1, and so we do not
// always desire to optimize all camera intrinsics. In many cases, the focal
// length is the only parameter we care to optimize.

// Users can specify which intrinsics to optimize by using a bitmask. For
// instance, FocalLength|PrincipalPoint will optimize the focal length and
// principal points.
enum class OptimizeIntrinsicsType
{
    None = 0x00,
    // Common
    FocalLength = 0x01,
    AspectRatio = 0x02,
    PrincipalPoint = 0x04,
    // Specialized
    Skew = 0x08,
    RadialDistortion = 0x10,
    TangentialDistortion = 0x20,

    All = FocalLength | AspectRatio | PrincipalPoint | Skew | RadialDistortion |
          TangentialDistortion,
    Common = FocalLength | AspectRatio | PrincipalPoint,
};
MAKE_FLAGS(OptimizeIntrinsicsType)

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

    virtual Type type() const = 0;

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

    void setFocalLength(double fx);
    double focalLength() const;
    inline auto focalLengthX() const { return focalLength(); }
    inline auto focalLengthY() const { return focalLengthX() * aspectRatio(); }
    inline auto fx() const { return focalLengthX(); }
    inline auto fy() const { return focalLengthY(); }

    // height / width
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

    // FIXME: Change this confusing name. return CONSTANT indexes!!!
    // Returns the indices of the parameters that will be optimized during BA
    virtual std::vector<int> constantParameterIndices(
        OptimizeIntrinsicsType flags) const;

    virtual void calibrationMatrix(Eigen::Matrix3d& matrix) const;
    inline Eigen::Matrix3d calibrationMatrix() const
    {
        Eigen::Matrix3d K;
        calibrationMatrix(K);
        return K;
    }
    inline auto matrixK() const { return calibrationMatrix(); }

    /// ------------------------- Point Mapping ----------------------------
    ///
    /// Static methods, for ceres. All the derived classes should implement this
    /// methods.
    // Given a (3D) point in the camera coordinate system, apply the camera
    // intrinsics to transform the point into pixel coordinates.
    // template <typename T>
    // static void spaceToPixel(const T* intrinsics, const T* point, T* pixel);

    // Given a pixel in the image coordinates, remove the effects of camera
    // intrinsics parameters and lens distortion to produce a point in the
    // camera coordinate system. The point output by this method is effectively
    // a ray in the direction of the pixel in the camera coordinate system.
    // template <typename T>
    // static void pixelToSpace(const T* intrinsics, const T* pixel, T* point);

    // Given an undistorted point, apply lens distortion to the point to get a
    // distorted point. The type of distortion (i.e. radial, tangential,
    // fisheye, etc.) will depend on the camera intrinsics model.
    // template <typename T>
    // static void distort(const T* intrinsics, const T* undistort, T*
    // distorted);

    // Given a distorted point, apply lens undistortion to the point to get an
    // undistorted point. The type of distortion (i.e. radial, tangential,
    // fisheye, etc.) will depend on the camera intrinsics model.
    // template <typename T>
    // static void undistort(const T* intrinsics, const T* distort, T*
    // undistorted);

    // TODO: Think of a smarter way, like generic programming

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
    // given in *normalized* coordinates such that the effects of camera
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
    virtual void print() const = 0;

protected:
    // NOTE: Dont use this string to serialize class
    std::string toLogString() const;

protected:
    std::vector<double> parameters_;
};

} // namespace thoht
