#pragma once

#include "camera_intrinsics.h"
#include <ceres/ceres.h>

namespace thoht {

// Brief:
// PinholeCameraModel
//
// Explanation:
// Intrinsics of the camera are modeled such that:
//
//  K = [f     s     cx]
//      [0    fy     cy]
//      [0     0      1]
//
// Extrinsic and intrinsic parameters transform the homogeneous 3D point X to
// the image point p such that:
//
//   p = R * (X[0..2] / X[3] - C);
//   p = p[0,1] / p[2];
//   r = p[0] * p[0] + p[1] * p[1];
//   d = 1 + k1 * r + k2 * r * r;
//   p *= d;
//   p = K * p;
//
class PinholeCameraModel final : public CameraIntrinsics
{
public:
    PinholeCameraModel();

    Type type() const override;

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // ----------------------- Parameters Access ----------------------
    //
    enum IntrinsicsIndex
    {
        Skew = ExtraIndex,
        K1,
        K2,

        IntrinsicsSize,
    };
    int numParameters() const override;

    void setSkew(double skew);
    double skew() const;

    void setRadialDistortion(double k1, double k2);
    double radialDistortion1() const;
    double radialDistortion2() const;
    inline auto k1() const { return radialDistortion1(); }
    inline auto k2() const { return radialDistortion2(); }

    std::vector<int> constantParameterIndices(
        OptimizeIntrinsicsType flags) const override;

    void calibrationMatrix(Eigen::Matrix3d& matrix) const override;

    // ---------------------- Point Mapping --------------------------------
    //
    template <typename T>
    static bool spaceToPixel(const T* intrinsics, const T* point, T* pixel);

    template <typename T>
    static bool pixelToSpace(const T* intrinsics, const T* pixel, T* point);

    template <typename T>
    static bool distort(const T* intrinsics, const T* undistort, T* distorted);

    template <typename T>
    static bool undistort(const T* intrinsics, const T* distort,
                          T* undistorted);

    template <typename T>
    static void calcDistortion(const T* intrinsics, const T* undistort,
                               T* radialDistortion);

    Eigen::Vector2d spaceToImage(const Eigen::Vector3d& point) const override
    {
        Eigen::Vector2d pixel;
        spaceToPixel(parameters(), point.data(), pixel.data());
        return pixel;
    }

    Eigen::Vector3d imageToSpace(const Eigen::Vector2d& pixel) const override
    {
        Eigen::Vector3d point;
        pixelToSpace(parameters(), pixel.data(), point.data());
        return point;
    }

    Eigen::Vector2d distort(const Eigen::Vector2d& undistorted) const override
    {
        Eigen::Vector2d distorted;
        distort(parameters(), undistorted.data(), distorted.data());
        return distorted;
    }

    Eigen::Vector2d undistort(const Eigen::Vector2d& distorted) const override
    {
        Eigen::Vector2d undistorted;
        undistort(parameters(), distorted.data(), undistorted.data());
        return undistorted;
    }

    void print() const override;
};

/// -------------------------- Implementation -------------------------------
///
template <typename T>
bool PinholeCameraModel::spaceToPixel(const T* intrinsics, const T* pt, T* px)
{
    // Normailized
    const T& depth = pt[2];
    const T normalized_pt[2]{pt[0] / depth, pt[1] / depth};

    // Apply radial distortion.
    T pt_d[2];
    PinholeCameraModel::distort(intrinsics, normalized_pt, pt_d);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    px[0] = fx * pt_d[0] + skew * pt_d[1] + cx;
    px[1] = fy * pt_d[1] + cy;

    return true;
}

template <typename T>
bool PinholeCameraModel::pixelToSpace(const T* intrinsics, const T* px, T* pt)
{
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    // Normalize the y coordinate first.
    T pt_d[2];
    pt_d[1] = (px[1] - cy) / fy;
    pt_d[0] = (px[0] - cx - pt_d[1] * skew) / fx;

    // Undo the radial distortion.
    PinholeCameraModel::undistort(intrinsics, pt_d, pt);
    pt[2] = T(1);

    return true;
}

template <typename T>
bool PinholeCameraModel::distort(const T* intrinsics, const T* pt_u, T* pt_d)
{
    T rd;
    calcDistortion(intrinsics, pt_u, &rd);

    pt_d[0] = pt_u[0] * rd;
    pt_d[1] = pt_u[1] * rd;

    return true;
}

template <typename T>
bool PinholeCameraModel::undistort(const T* intrinsics, const T* pt_d, T* pt_u)
{
    constexpr int kIterations{100};
    constexpr T kUndistortionEpsilon = T(1e-10);

    T prev_pt_u[2];
    pt_u[0] = pt_d[0];
    pt_u[1] = pt_d[1];
    for (int i{0}; i < kIterations; ++i) {
        prev_pt_u[0] = pt_u[0];
        prev_pt_u[1] = pt_u[1];

        T rd;
        calcDistortion(intrinsics, pt_u, &rd);

        pt_u[0] = pt_d[0] / rd;
        pt_u[1] = pt_d[1] / rd;

        if (ceres::abs(pt_u[0] - prev_pt_u[0]) < kUndistortionEpsilon &&
            ceres::abs(pt_u[1] - prev_pt_u[1]) < kUndistortionEpsilon) {
            break;
        }
    }

    return true;
}

template <typename T>
void PinholeCameraModel::calcDistortion(const T* intrinsics, const T* pt_u,
                                        T* rd)
{
    const T& k1 = intrinsics[K1];
    const T& k2 = intrinsics[K2];

    const T rr = pt_u[0] * pt_u[0] + pt_u[1] * pt_u[1];
    *rd = T(1) + rr * (k1 + k2 * rr);
}

} // namespace thoht
