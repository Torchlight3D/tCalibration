#pragma once

#include <ceres/ceres.h>

#include "camera_intrinsics.h"

namespace tl {

// Brief:
//
// Explanation:
//
// Reference:
class ExtendedUnifiedCameraModel final
    : public CameraIntrinsics_<ExtendedUnifiedCameraModel, 7>
{
    using Parent = CameraIntrinsics_<ExtendedUnifiedCameraModel, 7>;

public:
    ExtendedUnifiedCameraModel();

    inline static constexpr auto kType = CameraIntrinsicsType::ExtendedUnified;

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // ----------------------- Parameters Access -----------------------
    //
    enum IntrinsicsIndex
    {
        Skew = ExtraIndex,
        Alpha, // value range [0., 1.]
        Beta,  // value range [0., +inf]

        IntrinsicsSize,
    };
    static_assert(kNumParameters == IntrinsicsSize);

    void setSkew(double skew);
    double skew() const;

    void setDistortion(double alpha, double beta);
    double alpha() const;
    double beta() const;

    std::vector<int> fixedParameterIndices(
        OptimizeIntrinsicsType flags) const override;

    Eigen::Matrix3d calibrationMatrix() const override;

    // --------------------- Point Mapping  ------------------------------
    //
    template <typename T>
    static bool spaceToPixel(const T* intrinsics, const T* point, T* pixel);

    template <typename T>
    static bool pixelToSpace(const T* intrinsics, const T* pixel, T* point);

    template <typename T>
    static bool distortPoint(const T* intrinsics, const T undistort[3],
                             T* distorted);

    template <typename T>
    static bool undistortPoint(const T* intrinsics, const T* distort,
                               T undistorted[3]);

protected:
    std::string toLog() const override;
};

/// -------------------------- Implementation -------------------------------
///
template <typename T>
bool ExtendedUnifiedCameraModel::spaceToPixel(const T* intrinsics,
                                              const T* point, T* pixel)
{
    // No normalize to plane at depth 1

    // Apply distortion.
    T pt_d[2];
    bool result = distortPoint(intrinsics, point, pt_d);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    pixel[0] = fx * pt_d[0] + skew * pt_d[1] + cx;
    pixel[1] = fy * pt_d[1] + cy;

    return result;
}

template <typename T>
bool ExtendedUnifiedCameraModel::pixelToSpace(const T* intrinsics,
                                              const T* pixel, T* point)
{
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T& fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    // Normalize the y coordinate first.
    T pt_d[2];
    pt_d[1] = (pixel[1] - cy) / fy;
    pt_d[0] = (pixel[0] - cx - pt_d[1] * skew) / fx;

    return undistortPoint(intrinsics, pt_d, point);
}

template <typename T>
bool ExtendedUnifiedCameraModel::distortPoint(const T* intrinsics,
                                              const T* pt_u, T* pt_d)
{
    const T& alpha = intrinsics[Alpha];
    const T& beta = intrinsics[Beta];

    const T xx = pt_u[0] * pt_u[0];
    const T yy = pt_u[1] * pt_u[1];
    const T zz = pt_u[2] * pt_u[2];

    const T rr = xx + yy;
    const T rho2 = beta * rr + zz;
    const T rho = ceres::sqrt(rho2);

    const T norm = alpha * rho + (T(1.) - alpha) * pt_u[2];
    pt_d[0] = T(0.);
    pt_d[1] = T(0.);

    if (norm < T(1e-3)) {
        return true;
    }

    // Check that the point is in the upper hemisphere in case of ellipsoid
    if (alpha > T(0.5)) {
        const T zn = pt_u[2] / norm;
        const T C = (alpha - T(1)) / (alpha + alpha - T(1));
        if (zn < C) {
            return true;
        }
    }

    pt_d[0] = pt_u[0] / norm;
    pt_d[1] = pt_u[1] / norm;
    return true;
}

template <typename T>
bool ExtendedUnifiedCameraModel::undistortPoint(const T* intrinsics,
                                                const T* pt_d, T* pt_u)
{
    const T& alpha = intrinsics[Alpha];
    const T& beta = intrinsics[Beta];

    const T rr = pt_d[0] * pt_d[0] + pt_d[1] * pt_d[1];
    const T gamma = T(1) - alpha;

    if (alpha > T(0.5)) {
        if (rr >= T(1) / ((alpha - gamma) * beta)) {
            return false;
        }
    }

    const T tmp1 = (T(1) - alpha * alpha * beta * rr);
    const T tmp_sqrt = ceres::sqrt(T(1) - (alpha - gamma) * beta * rr);
    const T tmp2 = (alpha * tmp_sqrt + gamma);

    const T k = tmp1 / tmp2;

    T norm = ceres::sqrt(rr + k * k);
    if (norm < T(1e-12)) {
        norm = T(1e-12);
    }

    pt_u[0] = pt_d[0] / norm;
    pt_u[1] = pt_d[1] / norm;
    pt_u[2] = k / norm;

    return true;
}

} // namespace tl
