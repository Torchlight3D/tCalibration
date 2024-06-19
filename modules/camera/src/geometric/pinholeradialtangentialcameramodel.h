#pragma once

#include <tCore/AutoRegisterFactory>

#include "cameraintrinsics.h"

namespace tl {

// Brief:
// PinholeRadialTangentialCameraModel
//
// Explanation:
// Intrinsics of the camera are modeled such that:
//
//  K = [fx    s     cx]
//      [0     fy    cy]
//      [0     0      1]
//
// Extrinsic parameter transform the homogeneous 3D point X to the image point
// p such that:
//   p = R * (X[0..2] / X[3] - C);
//   p = p[0,1] / p[2];
//   r = p[0] * p[0] + p[1] * p[1];
//   d = 1 + k1 * r + k2 * r * r + k3 * r * r * r;
//   td[0] = 2 * t1 * p[0] * p[1] + t2 * (r + 2 * p[0] * p[0])
//   td[0] = 2 * t2 * p[0] * p[1] + t1 * (r + 2 * p[1] * p[1])
//   p[0] = p[0] * d + td[0];
//   p[1] = p[1] * d + td[1];
//   p = K * p;
//
class PinholeRadialTangentialCameraModel final
    : public CameraIntrinsics_<PinholeRadialTangentialCameraModel, 10>
{
    using Parent = CameraIntrinsics_<PinholeRadialTangentialCameraModel, 10>;
    REGISTER(kName, CameraIntrinsics);

public:
    PinholeRadialTangentialCameraModel();

    inline static constexpr char kName[35]{
        "PinholeRadialTangentialCameraModel"};
    inline static constexpr auto kType =
        CameraIntrinsicsType::PinholeRadialTangential;

    void setFromMetaData(const CameraMetaData& prior) override;
    CameraMetaData toMetaData() const override;

    // ----------------------- Parameters Access ------------------------
    //
    enum IntrinsicsIndex
    {
        Skew = ExtraIndex,
        K1,
        K2,
        K3,
        T1,
        T2,

        IntrinsicsSize,
    };
    static_assert(kNumParameters == IntrinsicsSize);

    void setSkew(double skew);
    double skew() const;

    void setRadialDistortion(double k1, double k2, double k3);
    double radialDistortion1() const;
    double radialDistortion2() const;
    double radialDistortion3() const;
    inline auto k1() const { return radialDistortion1(); }
    inline auto k2() const { return radialDistortion2(); }
    inline auto k3() const { return radialDistortion3(); }

    void setTangentialDistortion(double t1, double t2);
    double tangentialDistortion1() const;
    double tangentialDistortion2() const;
    inline auto t1() const { return tangentialDistortion1(); }
    inline auto t2() const { return tangentialDistortion2(); }

    std::vector<int> fixedParameterIndices(
        OptimizeIntrinsicsType flags) const override;

    Eigen::Matrix3d calibrationMatrix() const override;

    // ------------------------- Point Mapping ----------------------------
    //
    template <typename T>
    static bool spaceToPixel(const T* intrinsics, const T* point, T* pixel);

    template <typename T>
    static bool pixelToSpace(const T* intrinsics, const T* pixel, T* point);

    template <typename T>
    static bool distortPoint(const T* intrinsics, const T* undistort,
                             T* distorted);

    template <typename T>
    static bool undistortPoint(const T* intrinsics, const T* distort,
                               T* undistorted);

protected:
    std::string toLog() const override;

private:
    template <typename T>
    static void calcDistortion(const T* intrinsics, const T* undistort,
                               T* tangentialDistortion, T* radialDistortion);
};

/// -------------------------- Implementation -------------------------------
///
template <typename T>
bool PinholeRadialTangentialCameraModel::spaceToPixel(const T* intrinsics,
                                                      const T* point, T* pixel)
{
    // Get normalized pixel projection at image plane depth = 1.
    const T& depth = point[2];
    const T pt_norm[2] = {point[0] / depth, point[1] / depth};

    // Apply lens distortion.
    T pt_d[2];
    distortPoint(intrinsics, pt_norm, pt_d);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    pixel[0] = fx * pt_d[0] + skew * pt_d[1] + cx;
    pixel[1] = fy * pt_d[1] + cy;

    return true;
}

template <typename T>
bool PinholeRadialTangentialCameraModel::pixelToSpace(const T* intrinsics,
                                                      const T* pixel, T* point)
{
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    // Normalize the y first so that the skew may be applied appropriately.
    T pt_d[2];
    pt_d[1] = (pixel[1] - cy) / fy;
    pt_d[0] = (pixel[0] - cx - pt_d[1] * skew) / fx;

    undistortPoint(intrinsics, pt_d, point);
    point[2] = T(1.);

    return true;
}

template <typename T>
bool PinholeRadialTangentialCameraModel::distortPoint(const T* intrinsics,
                                                      const T* pt_u, T* pt_d)
{
    T td[2];
    T rd[1];
    calcDistortion(intrinsics, pt_u, td, rd);

    pt_d[0] = pt_u[0] * rd[0] + td[0];
    pt_d[1] = pt_u[1] * rd[0] + td[1];

    return true;
}

template <typename T>
bool PinholeRadialTangentialCameraModel::undistortPoint(const T* intrinsics,
                                                        const T* pt_d, T* pt_u)
{
    constexpr int kIterations{100};
    const T kUndistortionEpsilon = T(1e-10);

    T prev_pt_u[2];
    pt_u[0] = pt_d[0];
    pt_u[1] = pt_d[1];
    for (int i{0}; i < kIterations; ++i) {
        prev_pt_u[0] = pt_u[0];
        prev_pt_u[1] = pt_u[1];

        T td[2];
        T rd[1];
        calcDistortion(intrinsics, pt_u, td, rd);

        pt_u[0] = (pt_d[0] - td[0]) / rd[0];
        pt_u[1] = (pt_d[1] - td[1]) / rd[0];

        if (abs(pt_u[0] - prev_pt_u[0]) < kUndistortionEpsilon &&
            abs(pt_u[1] - prev_pt_u[1]) < kUndistortionEpsilon) {
            break;
        }
    }

    return true;
}

template <typename T>
void PinholeRadialTangentialCameraModel::calcDistortion(const T* intrinsics,
                                                        const T* pt_u, T* td,
                                                        T* rd)
{
    const T& k1 = intrinsics[K1];
    const T& k2 = intrinsics[K2];
    const T& k3 = intrinsics[K3];
    const T& t1 = intrinsics[T1];
    const T& t2 = intrinsics[T2];

    const T xx = pt_u[0] * pt_u[0];
    const T yy = pt_u[1] * pt_u[1];
    const T xy = pt_u[0] * pt_u[1];

    // Radius of the undistorted point.
    const T rr = xx + yy;

    // Radial distortion
    // r = 1 + k1 * r^2 + k2 * r^4 + k3 * r^6
    rd[0] = T(1) + k1 * rr + k2 * rr * rr + k3 * rr * rr * rr;

    // Tangential distortion
    // x = t2 * (r^2 + 2 * x^2) + 2 * t1 * x * y
    // y = t1 * (r^2 + 2 * y^2) + 2 * t2 * x * y
    td[0] = t2 * (rr + T(2) * xx) + T(2) * t1 * xy;
    td[1] = t1 * (rr + T(2) * yy) + T(2) * t2 * xy;
}

} // namespace tl
