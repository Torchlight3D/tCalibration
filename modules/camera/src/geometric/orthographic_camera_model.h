#pragma once

#include <ceres/ceres.h>

#include "camera_intrinsics.h"

namespace tl {

// Brief:
//
// Explanation:
//
// Reference:
class OrthographicCameraModel final : public CameraIntrinsics
{
public:
    OrthographicCameraModel();

    constexpr Type type() const override { return Type::Orthographic; }

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // ---------------------- Parameters Access --------------------------
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

    Eigen::Matrix3d calibrationMatrix() const override;

    bool isValid() const override;

    // ------------------------- Point Mapping ----------------------------
    //
    template <typename T>
    static bool spaceToPixel(const T* intrinsics, const T* point, T* pixel);

    template <typename T>
    static bool pixelToSpace(const T* intrinsics, const T* pixel, T* point);

    template <typename T>
    static bool isUnprojectable(const T* intrinsics, const T* pixel);

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
        OrthographicCameraModel::undistort(parameters(), distorted.data(),
                                           undistorted.data());
        return undistorted;
    }

protected:
    std::string toLog() const override;
};

/// -------------------------- Implementation -------------------------------
///
template <typename T>
bool OrthographicCameraModel::spaceToPixel(const T* intrinsics, const T* point,
                                           T* pixel)
{
    // Apply radial distortion.
    T distorted_pixel[2];
    OrthographicCameraModel::distort(intrinsics, point, distorted_pixel);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    pixel[0] = fx * distorted_pixel[0] + skew * distorted_pixel[1] + cx;
    pixel[1] = fy * distorted_pixel[1] + cy;

    return true;
}

template <typename T>
bool OrthographicCameraModel::pixelToSpace(const T* intrinsics, const T* pixel,
                                           T* point)
{
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    // Normalize the y coordinate first.
    T pt_d[2];
    pt_d[1] = pixel[1] - cy;
    pt_d[0] = pixel[0] - cx - pt_d[1] * skew;
    pt_d[0] /= fx;
    pt_d[1] /= fy;

    // Undo the radial distortion.
    OrthographicCameraModel::undistort(intrinsics, pt_d, point);
    point[2] = T(1.);
    return true;
}

template <typename T>
bool OrthographicCameraModel::isUnprojectable(const T* intrinsics,
                                              const T* pixel)
{
    return true;
}

template <typename T>
bool OrthographicCameraModel::distort(const T* intrinsics, const T* pt_u,
                                      T* pt_d)
{
    T rd;
    calcDistortion(intrinsics, pt_u, &rd);

    pt_d[0] = pt_u[0] * rd;
    pt_d[1] = pt_u[1] * rd;

    return true;
}

template <typename T>
bool OrthographicCameraModel::undistort(const T* intrinsics, const T* pt_d,
                                        T* pt_u)
{
    constexpr int kIterations{100};
    const T kUndistortionEpsilon = T(1e-16);

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
void OrthographicCameraModel::calcDistortion(const T* intrinsics, const T* pt_u,
                                             T* rd)
{
    const T& k1 = intrinsics[K1];
    const T& k2 = intrinsics[K2];

    const T rr = pt_u[0] * pt_u[0] + pt_u[1] * pt_u[1];
    *rd = T(1.) + rr * (k1 + k2 * rr);
}

} // namespace tl
