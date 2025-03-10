﻿#pragma once

#include <tCore/AutoRegisterFactory>

#include "cameraintrinsics.h"

namespace tl {

// Brief:
//
// Explanation:
//
// Reference:
class DivisionUndistortionCameraModel final
    : public CameraIntrinsics_<DivisionUndistortionCameraModel, 5>
{
    using Parent = CameraIntrinsics_<DivisionUndistortionCameraModel, 5>;
    REGISTER(kName, CameraIntrinsics);

public:
    DivisionUndistortionCameraModel();

    inline static constexpr char kName[32]{"DivisionUndistortionCameraModel"};
    inline static constexpr auto kType =
        CameraIntrinsicsType::DivisionUndistortion;

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // ---------------------- Parameters Access --------------------------
    //
    enum
    {
        K = ExtraIndex,

        IntrinsicsSize,
    };
    static_assert(kNumParameters == IntrinsicsSize);

    void setRadialDistortion(double k);
    double radialDistortion1() const;
    inline auto k() const { return radialDistortion1(); }

    std::vector<int> fixedParameterIndices(
        OptimizeIntrinsicsType flags) const override;

    // ---------------------- Point Mapping --------------------------------
    //
    template <typename T>
    static bool spaceToPixel(const T* params, const T* point, T* pixel);

    template <typename T>
    static bool pixelToSpace(const T* params, const T* pixel, T* point);

    template <typename T>
    inline static bool distortPoint(const T* params, const T* pixel,
                                    T* distorted)
    {
        const auto& k = params[K];
        return distortImpl(k, pixel, distorted);
    }

    template <typename T>
    static bool undistortPoint(const T* params, const T* pixel, T* undistorted);

    // Extras
    template <typename T>
    static bool CameraToUndistortedPixelCoordinates(const T* params,
                                                    const T* point,
                                                    T* undistorted);

    template <typename T>
    static bool DistortedPixelToUndistortedPixel(const T* params,
                                                 const T* distorted,
                                                 T* undistorted);

protected:
    std::string toLog() const override;

private:
    template <typename T>
    static bool distortImpl(T k, const T* pixel, T* distorted);
};

/// -------------------------- Implementation -------------------------------
///
template <typename T>
bool DivisionUndistortionCameraModel::spaceToPixel(const T* intrinsics,
                                                   const T* pt, T* px)
{
    // Get normalized pixel projection at image plane depth = 1.
    const T& depth = pt[2];
    const T px_norm[2] = {pt[0] / depth, pt[1] / depth};

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;

    // Apply the focal length and aspect ratio.
    T px_u[2];
    px_u[0] = fx * px_norm[0];
    px_u[1] = fy * px_norm[1];

    // Apply radial distortion.
    distortPoint(intrinsics, px_u, px);

    // Add the principal point.
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];

    px[0] += cx;
    px[1] += cy;

    return true;
}

template <typename T>
bool DivisionUndistortionCameraModel::pixelToSpace(const T* intrinsics,
                                                   const T* px, T* pt)
{
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];

    // Normalize the y coordinate first.
    T pt_d[2];
    pt_d[0] = px[0] - cx;
    pt_d[1] = px[1] - cy;

    // Undo the radial distortion.
    undistortPoint(intrinsics, pt_d, pt);

    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;

    pt[0] /= fx;
    pt[1] /= fy;
    pt[2] = T(1.);

    return true;
}

template <typename T>
bool DivisionUndistortionCameraModel::distortImpl(T k, const T* pt_u, T* pt_d)
{
    // From distorted to undistort
    // x_u = x_d / (1 + k * r_d^2)
    // y_u = y_d / (1 + k * r_d^2)
    //
    // Thus, from undistort to distorted is given by
    // x_d = x_u * (1 - sqrt(1 - 4 * k * r_u^2)) / (2 * k * r_u^2)
    // y_d = y_u * (1 - sqrt(1 - 4 * k * r_u^2)) / (2 * k * r_u^2)
    const T rr = pt_u[0] * pt_u[0] + pt_u[1] * pt_u[1];
    const T denom = T(2) * k * rr;
    const T inner_sqrt = T(1) - T(4) * k * rr;

    // If the denominator is nearly zero then we can evaluate the distorted
    // coordinates as k or r_u^2 goes to zero. Both evaluate to the identity.
    const T kVerySmallNumber(1e-15);
    if (abs(denom) < kVerySmallNumber || inner_sqrt < T(0)) {
        pt_d[0] = pt_u[0];
        pt_d[1] = pt_u[1];
    }
    else {
        const T scale = (T(1) - sqrt(inner_sqrt)) / denom;
        pt_d[0] = pt_u[0] * scale;
        pt_d[1] = pt_u[1] * scale;
    }

    return true;
}

template <typename T>
bool DivisionUndistortionCameraModel::undistortPoint(const T* intrinsics,
                                                     const T* pt_d, T* pt_u)
{
    const T rr = pt_d[0] * pt_d[0] + pt_d[1] * pt_d[1];

    // From distorted to undistort
    // x_u = x_d / (1 + k * r_d^2)
    //
    // While care must be taken to avoid dividing by a zero term, this is almost
    // never the case in practice so we ignore it here.
    const T& k = intrinsics[K];
    const T rd = T(1.) / (T(1.) + k * rr);
    pt_u[0] = pt_d[0] * rd;
    pt_u[1] = pt_d[1] * rd;

    return true;
}

template <typename T>
bool DivisionUndistortionCameraModel::CameraToUndistortedPixelCoordinates(
    const T* intrinsics, const T* pt, T* pt_u)
{
    // Get normalized pixel projection at image plane depth = 1.
    const T& depth = pt[2];
    const T px_norm[2] = {pt[0] / depth, pt[1] / depth};

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];

    // Apply the focal length and aspect ratio.
    pt_u[0] = fx * px_norm[0] + cx;
    pt_u[1] = fy * px_norm[1] + cy;

    return true;
}

// Transform the distorted pixel to the undistorted pixel.
template <typename T>
bool DivisionUndistortionCameraModel::DistortedPixelToUndistortedPixel(
    const T* intrinsics, const T* pt_d, T* pt_u)
{
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];

    // Normalize the y coordinate first.
    T pt_d_norm[2];
    pt_d_norm[0] = pt_d[0] - cx;
    pt_d_norm[1] = pt_d[1] - cy;

    // Undo the radial distortion.
    distortPoint(intrinsics, pt_d_norm, pt_u);

    pt_u[0] += cx;
    pt_u[1] += cy;

    return true;
}

} // namespace tl
