#pragma once

#include <ceres/ceres.h>

#include "camera_intrinsics.h"

namespace tl {

// Brief:
//
// Explanation:
//
// Reference:

// FIXME: Use FovCameraModel
class FovCameraModel final : public CameraIntrinsics_<FovCameraModel, 5>
{
    using Parent = CameraIntrinsics_<FovCameraModel, 5>;

public:
    FovCameraModel();

    inline static constexpr auto kType = CameraIntrinsicsType::Fov;

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // --------------------- Parameters Access ------------------------------
    //
    enum IntrinsicsIndex
    {
        Omega = ExtraIndex,

        IntrinsicsSize,
    };
    static_assert(kNumParameters == IntrinsicsSize);

    inline static constexpr auto kDefaultOmega{0.75};
    void setRadialDistortion(double omega);
    double radialDistortion1() const;
    inline auto omega() const { return radialDistortion1(); }

    std::vector<int> fixedParameterIndices(
        OptimizeIntrinsicsType flags) const override;

    // ---------------------- Point Mapping --------------------------------
    //
    template <typename T>
    static bool spaceToPixel(const T* params, const T* point, T* pixel);

    template <typename T>
    static bool pixelToSpace(const T* params, const T* pixel, T* point);

    template <typename T>
    static bool distortPoint(const T* params, const T* pixel, T* distorted);

    template <typename T>
    static bool undistortPoint(const T* params, const T* pixel, T* undistorted);

protected:
    std::string toLog() const override;
};

/// ---------------------- Implementation -----------------------------------
///
template <typename T>
bool FovCameraModel::spaceToPixel(const T* intrinsics, const T* pt, T* px)
{
    // Normalize
    const T& depth = pt[2];
    const T px_norm[2]{pt[0] / depth, pt[1] / depth};

    // Apply distortion
    T px_d[2];
    distortPoint(intrinsics, px_norm, px_d);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& xy = intrinsics[Cy];

    px[0] = fx * px_d[0] + cx;
    px[1] = fy * px_d[1] + xy;

    return true;
}

template <typename T>
bool FovCameraModel::pixelToSpace(const T* intrinsics, const T* px, T* pt)
{
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];

    T pt_d[2];
    pt_d[0] = (px[0] - cx) / fx;
    pt_d[1] = (px[1] - cy) / fy;

    undistortPoint(intrinsics, pt_d, pt);
    pt[2] = T(1.);

    return true;
}

template <typename T>
bool FovCameraModel::distortPoint(const T* intrinsics, const T* pt_u, T* pt_d)
{
    static const T kVerySmallNumber(1e-3);

    const T& omega = intrinsics[Omega];

    const T rr = pt_u[0] * pt_u[0] + pt_u[1] * pt_u[1];

    // If omega is very small then we need to approximate the distortion using
    // the Taylor series so that bundle adjustment will still be able to compute
    // a gradient and can appropriately optimize the distortion parameter.
    T r_d;
    if (omega < kVerySmallNumber) {
        // Derivation of this case with Matlab borrowed from COLMAP:
        // https://github.com/colmap/colmap/blob/master/src/base/camera_models.h#L1107
        //
        // syms radius omega;
        // factor(radius) = tan(radius * omega) / ...
        //                  (radius * 2*tan(omega/2));
        // simplify(taylor(factor, omega, 'order', 3))
        r_d = T(1) + (omega * omega * rr) / T(3) - omega * omega / T(12);
    }
    else if (rr < kVerySmallNumber) {
        // Derivation of this case with Matlab borrowed from COLMAP:
        // https://github.com/colmap/colmap/blob/master/src/base/camera_models.h#L1107
        //
        // syms radius omega;
        // factor(radius) = tan(radius * omega) / ...
        //                  (radius * 2*tan(omega/2));
        // simplify(taylor(factor, radius, 'order', 3))
        const T tan_half_omega = ceres::tan(omega / T(2));
        r_d = (-T(2) * tan_half_omega *
               (T(4) * rr * tan_half_omega * tan_half_omega - T(3))) /
              (T(3) * omega);
    }
    else {
        // Compute the radius of the distorted image point based on the FOV
        // model equations.
        const T r_u = ceres::sqrt(rr);
        r_d =
            ceres::atan(T(2) * r_u * ceres::tan(omega / T(2))) / (r_u * omega);
    }

    // Compute the radius of the distorted image point based on the FOV model
    // equations.
    pt_d[0] = r_d * pt_u[0];
    pt_d[1] = r_d * pt_u[1];

    return true;
}

template <typename T>
bool FovCameraModel::undistortPoint(const T* intrinsics, const T* pt_d, T* pt_u)
{
    const T& omega = intrinsics[Omega];
    const T omega_2 = omega * omega;

    const T rr = pt_d[0] * pt_d[0] + pt_d[1] * pt_d[1];

    // If omega is very small then we need to approximate the distortion using
    // the Taylor series so that bundle adjustment will still be able to compute
    // a gradient and can appropriately optimize the distortion parameter.
    T r_u;
    const T kVerySmallNumber(1e-3);
    if (omega < kVerySmallNumber) {
        // Derivation of this case with Matlab borrowed from COLMAP:
        // https://github.com/colmap/colmap/blob/master/src/base/camera_models.h#L1146
        //
        // syms radius omega;
        // factor(radius) = tan(radius * omega) / ...
        //                  (radius * 2*tan(omega/2));
        // simplify(taylor(factor, omega, 'order', 3))
        r_u = (omega_2 * rr) / T(3) - omega_2 / T(12) + T(1);
    }
    else if (rr < kVerySmallNumber) {
        // Derivation of this case with Matlab borrowed from COLMAP:
        // https://github.com/colmap/colmap/blob/master/src/base/camera_models.h#L1146
        //
        // syms radius omega;
        // factor(radius) = tan(radius * omega) / ...
        //                  (radius * 2*tan(omega/2));
        // simplify(taylor(factor, radius, 'order', 3))
        r_u =
            (omega * (omega_2 * rr + T(3))) / (T(6) * ceres::tan(omega / T(2)));
    }
    else {
        // Compute the radius of the distorted image point based on the FOV
        // model equations.
        const T rd = ceres::sqrt(rr);
        r_u = ceres::tan(rd * omega) / (T(2) * rd * ceres::tan(omega / T(2)));
    }

    pt_u[0] = r_u * pt_d[0];
    pt_u[1] = r_u * pt_d[1];

    return true;
}

} // namespace tl
