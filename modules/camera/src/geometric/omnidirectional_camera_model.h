#pragma once

#include <ceres/ceres.h>

#include "camera_intrinsics.h"

namespace tl {

// Brief:
//
// Explanation:
//
// Reference:
class OmnidirectionalCameraModel final
    : public CameraIntrinsics_<OmnidirectionalCameraModel, 9>
{
    using Parent = CameraIntrinsics_<OmnidirectionalCameraModel, 9>;

public:
    OmnidirectionalCameraModel();

    inline static constexpr auto kType = CameraIntrinsicsType::Omnidirectional;

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // --------------------- Parameters Access ------------------------------
    //
    enum IntrinsicsIndex
    {
        Xi = ExtraIndex,
        K1,
        K2,
        P1,
        P2,

        IntrinsicsSize,
    };
    static_assert(kNumParameters == IntrinsicsSize);

    // There's no "focal length" in Omnidirectional camera model.
    // To keep the interface consistent, we treat gamma1/gamma2 as fx/fy.
    inline void setGamma(double gamma1, double gamma2)
    {
        setFocalLength(gamma1);
        setAspectRatio(gamma2 / gamma1);
    }
    inline auto gamma1() const { return focalLengthX(); }
    inline auto gamma2() const { return focalLengthY(); }

    void setMirrorDistortion(double xi);
    double mirrorDistortion() const;
    inline auto xi() const { return mirrorDistortion(); }

    void setRadialDistortion(double k1, double k2);
    double radialDistortion1() const;
    double radialDistortion2() const;
    inline auto k1() const { return radialDistortion1(); }
    inline auto k2() const { return radialDistortion2(); }

    void setTangentialDistortion(double p1, double p2);
    double tangentialDistortion1() const;
    double tangentialDistortion2() const;
    inline auto p1() const { return tangentialDistortion1(); }
    inline auto p2() const { return tangentialDistortion2(); }

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

private:
    // Apply distortion to undistort point to get distorted point
    // pt_d = pt_u + distortion
    template <typename T>
    static void calcDistortion(const T* intrinsics, const T* undistort,
                               T* distortion);
};

/// ---------------------- Implementation -----------------------------------
///
template <typename T>
bool OmnidirectionalCameraModel::spaceToPixel(const T* intrinsics, const T* pt,
                                              T* px)
{
    const T& xi = intrinsics[Xi];

    // Normalize
    const T norm = Eigen::Map<const Eigen::Vector3<T>>(pt).norm();
    const T& depth = pt[2] + xi * norm;
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
bool OmnidirectionalCameraModel::pixelToSpace(const T* intrinsics, const T* px,
                                              T* pt)
{
    // Unproject
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];

    const T Kinv_11 = T(1) / fx;
    const T Kinv_22 = T(1) / fy;
    const T Kinv_13 = -cx / fx;
    const T Kinv_23 = -cy / fy;

    T pt_d[2];
    pt_d[0] = Kinv_11 * px[0] + Kinv_13;
    pt_d[1] = Kinv_22 * px[1] + Kinv_23;

    // Undistort
    T pt_u[2];
    undistortPoint(intrinsics, pt_d, pt_u);

    const T& xi = intrinsics[Xi];
    const T& x_u = pt_u[0];
    const T& y_u = pt_u[1];
    const T rho2 = x_u * x_u + y_u * y_u;

    pt[0] = x_u;
    pt[1] = y_u;
    if (xi == T(1)) {
        pt[2] = (T(1) - rho2) / T(2);
    }
    else {
        pt[2] = T(1) - xi * (rho2 + T(1)) /
                           (xi + ceres::sqrt(T(1) + (T(1) - xi * xi) * rho2));
    }

    return true;
}

template <typename T>
bool OmnidirectionalCameraModel::distortPoint(const T* intrinsics,
                                              const T* pt_u, T* pt_d)
{
    T dist[2];
    calcDistortion(intrinsics, pt_u, dist);

    pt_d[0] = pt_u[0] + dist[0];
    pt_d[1] = pt_u[1] + dist[1];

    return true;
}

template <typename T>
bool OmnidirectionalCameraModel::undistortPoint(const T* intrinsics,
                                                const T* pt_d, T* pt_u)
{
    constexpr bool kRecursive{true};
    if constexpr (kRecursive) {
        constexpr int kIterationCount{8};

        T prev_pt_u[2];
        pt_u[0] = pt_d[0];
        pt_u[1] = pt_d[1];
        for (int i{0}; i < kIterationCount; ++i) {
            prev_pt_u[0] = pt_u[0];
            prev_pt_u[1] = pt_u[1];

            T td[2];
            calcDistortion(intrinsics, pt_u, td);

            pt_u[0] = pt_d[0] - td[0];
            pt_u[1] = pt_d[1] - td[1];

            // TODO: Converge condition
        }
    }
    else {
        // Inverse distortion model, by Heikkila(???)
        const T& k1 = intrinsics[K1];
        const T& k2 = intrinsics[K2];
        const T& p1 = intrinsics[P1];
        const T& p2 = intrinsics[P2];

        const T& x = pt_d[0];
        const T& y = pt_d[1];
        const T xx = x * x;
        const T yy = y * y;
        const T xy = x * y;
        const T rho2 = xx + yy;
        const T rho4 = rho2 * rho2;
        const T rd = k1 * rho2 + k2 * rho4;
        const T dx = x * rd + p2 * (rho2 + T(2) * xx) + T(2) * p1 * xy;
        const T dy = y * rd + p1 * (rho2 + T(2) * yy) + T(2) * p2 * xy;
        const T deno = T(1) / (T(1) + T(4) * k1 * rho2 + T(6) * k2 * rho4 +
                               T(8) * p1 * y + T(8) * p2 * x);

        pt_u[0] = x - dx * deno;
        pt_u[1] = y - dy * deno;
    }

    return false;
}

template <typename T>
void OmnidirectionalCameraModel::calcDistortion(const T* intrinsics,
                                                const T* pt_u, T* dist)
{
    const T& k1 = intrinsics[K1];
    const T& k2 = intrinsics[K2];
    const T& p1 = intrinsics[P1];
    const T& p2 = intrinsics[P2];

    const T& x_u = pt_u[0];
    const T& y_u = pt_u[1];
    const T xx = x_u * x_u;
    const T yy = y_u * y_u;
    const T xy = x_u * y_u;
    const T rho2 = xx + yy;
    const T rd = k1 * rho2 + k2 * rho2 * rho2;

    dist[0] = x_u * rd + T(2) * p1 * xy + p2 * (rho2 + T(2) * xx);
    dist[1] = y_u * rd + T(2) * p2 * xy + p1 * (rho2 + T(2) * yy);
}

} // namespace tl
