#pragma once

#include <memory>
#include <string>

#include <glog/logging.h>

#include <magic_enum/magic_enum.hpp>

#include <tCamera/Types>

#include "camera_meta_data.h"
#include "../util_camera_matrix.h"

namespace tl {

enum class CameraIntrinsicsType
{
    Pinhole = 0,
    PinholeRadialTangential,
    Fisheye, // a.k.a. KannalaBrandt(KB), Equidistant
    Fov,
    DivisionUndistortion,
    DoubleSphere,
    ExtendedUnified,
    Orthographic,
    Omnidirectional, // a.k.a Mei
};

class CameraIntrinsics
{
public:
    using Ptr = std::shared_ptr<CameraIntrinsics>;
    using ConstPtr = std::shared_ptr<const CameraIntrinsics>;

    virtual ~CameraIntrinsics() = default;

    virtual CameraIntrinsics& operator=(const CameraIntrinsics& o) = 0;

    // TODO: Use auto registration
    static Ptr create(CameraIntrinsicsType type);

    //////////////////////////// Properties ////////////////////////////
    virtual CameraIntrinsicsType type() const = 0;
    virtual std::string name() const = 0;

    //////////////////////////// Parameters ////////////////////////////

    virtual void setFromMetaData(const CameraMetaData& meta) = 0;
    virtual CameraMetaData toMetaData() const = 0;

    virtual bool setParameter(int index, double value) = 0;
    virtual double parameter(int index) const = 0;
    virtual const double* parameters() const = 0;
    virtual double* rParameters() = 0;
    virtual std::vector<double> asVector() const = 0;
    virtual size_t numParameters() const = 0;

    enum IntrinsicsIndex
    {
        Fx = 0,
        YX,
        Cx,
        Cy,

        ExtraIndex, // Learn from Qt::UserRole
    };

    // [fx, fy(aspect ratio), cx, cy] are essential paramters
    virtual void setFocalLength(double fx) = 0;
    virtual double focalLength() const = 0;
    inline auto focalLengthX() const { return focalLength(); }
    inline auto focalLengthY() const { return focalLengthX() * aspectRatio(); }
    inline auto fx() const { return focalLengthX(); }
    inline auto fy() const { return focalLengthY(); }

    // Height / Width <=> Y/X
    virtual void setAspectRatio(double aspectRatio) = 0;
    virtual double aspectRatio() const = 0;

    virtual void setPrincipalPoint(double cx, double cy) = 0;
    virtual double principalPointX() const = 0;
    virtual double principalPointY() const = 0;
    inline auto cx() const { return principalPointX(); }
    inline auto cy() const { return principalPointY(); }

    virtual std::vector<int> fixedParameterIndices(
        OptimizeIntrinsicsType type) const = 0;

    virtual Eigen::Matrix3d calibrationMatrix() const
    {
        return intrinsicsToCalibrationMatrix(fx(), 0., aspectRatio(), cx(),
                                             cy());
    }
    inline void calibrationMatrix(Eigen::Matrix3d& K) const
    {
        K = calibrationMatrix();
    }
    inline auto matrixK() const { return calibrationMatrix(); }

    // Scale by image size
    virtual void scale(double s) = 0;

    //////////////////////////// Point Mapping ////////////////////////////

    // Projects the 3D point in the camera coordinate system into the image
    // plane and distorts the point according to the radial distortion
    // parameters.
    virtual Eigen::Vector2d spaceToImage(
        const Eigen::Vector3d& point) const = 0;

    // Converts image pixel coordinates to normalized coordinates in the camera
    // coordinates by removing the effect of camera intrinsics/calibration.
    virtual Eigen::Vector3d imageToSpace(
        const Eigen::Vector2d& pixel) const = 0;

    virtual Eigen::Vector2d distort(const Eigen::Vector2d& pixel) const = 0;

    virtual Eigen::Vector2d undistort(const Eigen::Vector2d& pixel) const = 0;

    // TODO:
    // 1. Jacobians
    // 2. Distortion map

    /// Debug
    friend std::ostream& operator<<(std::ostream& os,
                                    const CameraIntrinsics& cam);

protected:
    virtual std::string toLog() const = 0;
};

template <typename Derived, size_t ParameterCount>
class CameraIntrinsics_ : public CameraIntrinsics
{
    using _MyType = CameraIntrinsics_<Derived, ParameterCount>;

public:
    CameraIntrinsics_() : CameraIntrinsics()
    {
        setFocalLength(1.);
        setAspectRatio(1.);
        setPrincipalPoint(0., 0.);
    }
    virtual ~CameraIntrinsics_() = default;

    CameraIntrinsics& operator=(const CameraIntrinsics& o) final
    {
        CHECK(type() == o.type())
            << "Cannot assign a " << o.name() << " to a " << name();

        static_cast<_MyType*>(this)->params_ =
            static_cast<const _MyType*>(&o)->params_;

        return *this;
    }

    //////////////////////////// Properties ////////////////////////////

    CameraIntrinsicsType type() const final { return Derived::kType; }

    std::string name() const final
    {
        return std::string{magic_enum::enum_name(type())};
    }

    //////////////////////////// Parameters ////////////////////////////

    void setFromMetaData(const CameraMetaData& meta) override
    {
        if (meta.focal_length.is_set) {
            setFocalLength(meta.focal_length.value[0]);
        }
        else if (meta.image_width != 0. && meta.image_height != 0.) {
            setFocalLength(1.2 * std::max(meta.image_width, meta.image_height));
        }

        if (meta.principal_point.is_set) {
            setPrincipalPoint(meta.principal_point.value[0],
                              meta.principal_point.value[1]);
        }
        else if (meta.image_width != 0. && meta.image_height != 0.) {
            setPrincipalPoint(meta.image_width / 2., meta.image_height / 2.);
        }

        if (meta.aspect_ratio.is_set) {
            setAspectRatio(meta.aspect_ratio.value[0]);
        }
    }

    CameraMetaData toMetaData() const override
    {
        CameraMetaData meta;
        meta.camera_intrinsics_model_type = magic_enum::enum_name(type());
        meta.focal_length.is_set = true;
        meta.focal_length.value[0] = focalLength();
        meta.principal_point.is_set = true;
        meta.principal_point.value[0] = principalPointX();
        meta.principal_point.value[1] = principalPointY();
        meta.aspect_ratio.is_set = true;
        meta.aspect_ratio.value[0] = aspectRatio();

        return meta;
    }

    bool setParameter(int index, double value) final
    {
        DCHECK_GE(index, 0);
        DCHECK_LT(index, params_.size());
        params_[index] = value;
        return true;
    }

    double parameter(int index) const final
    {
        DCHECK_GE(index, 0);
        DCHECK_LT(index, params_.size());
        return params_[index];
    }

    const double* parameters() const final { return params_.data(); }

    double* rParameters() final { return params_.data(); }

    std::vector<double> asVector() const final
    {
        return {params_.cbegin(), params_.cend()};
    }

    inline static constexpr auto kNumParameters = ParameterCount;
    static_assert(kNumParameters >= 4,
                  "Camera intrinsics should have at least 4 parameters. "
                  "fx, fy(or aspect ratio), cx, cy.");

    size_t numParameters() const final { return kNumParameters; }

    std::vector<int> fixedParameterIndices(
        OptimizeIntrinsicsType flags) const override
    {
        using Type = OptimizeIntrinsicsType;

        if (flags == Type::All) {
            return {};
        }

        std::vector<int> indices;
        if ((flags & Type::FocalLength) == Type::None) {
            indices.emplace_back(Fx);
        }
        if ((flags & Type::AspectRatio) == Type::None) {
            indices.emplace_back(YX);
        }
        if ((flags & Type::PrincipalPoint) == Type::None) {
            indices.emplace_back(Cx);
            indices.emplace_back(Cy);
        }

        return indices;
    }

    void setFocalLength(double fx) final
    {
        CHECK_GT(fx, 0.) << "Invalid focal length value."
                            "Focal length must be greater than 0.0.";
        setParameter(Fx, fx);
    }

    double focalLength() const final { return parameter(Fx); }

    void setAspectRatio(double aspectRatio) final
    {
        CHECK_GT(aspectRatio, 0.0) << "Invalid aspect ratio."
                                      "Aspect ratio must be greater than 0.0.";
        setParameter(YX, aspectRatio);
    }

    double aspectRatio() const final { return parameter(YX); }

    void setPrincipalPoint(double cx, double cy) final
    {
        setParameter(Cx, cx);
        setParameter(Cy, cy);
    }

    double principalPointX() const final { return parameter(Cx); }

    double principalPointY() const final { return parameter(Cy); }

    void scale(double s) override
    {
        setFocalLength(fx() * s);
        setPrincipalPoint(s * (cx() + 0.5) - 0.5, s * (cy() + 0.5) - 0.5);
    }

    //////////////////////////// Point Mapping ////////////////////////////

    Eigen::Vector2d spaceToImage(const Eigen::Vector3d& point) const final
    {
        Eigen::Vector2d pix;
        Derived::spaceToPixel(params_.data(), point.data(), pix.data());
        return pix;
    }

    Eigen::Vector3d imageToSpace(const Eigen::Vector2d& pixel) const final
    {
        Eigen::Vector3d ray;
        Derived::pixelToSpace(params_.data(), pixel.data(), ray.data());
        return ray;
    }

    Eigen::Vector2d distort(const Eigen::Vector2d& pixel) const final
    {
        Eigen::Vector2d distorted;
        Derived::distortPoint(params_.data(), pixel.data(), distorted.data());
        return distorted;
    }

    Eigen::Vector2d undistort(const Eigen::Vector2d& pixel) const final
    {
        Eigen::Vector2d undistorted;
        Derived::undistortPoint(params_.data(), pixel.data(),
                                undistorted.data());
        return undistorted;
    }

protected:
    std::string toLog() const override
    {
        std::ostringstream oss;
        oss << "Camera model type: " << name()
            << "\n"
               "Focal length (pixel): "
            << fx() << ", " << fy()
            << "\n"
               "Aspect ratio (height/width): "
            << aspectRatio()
            << "\n"
               "Principal point: "
            << cx() << ", " << cy();

        return oss.str();
    }

protected:
    std::array<double, kNumParameters> params_;
};

} // namespace tl
