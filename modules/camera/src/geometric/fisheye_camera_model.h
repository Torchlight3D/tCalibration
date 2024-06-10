#pragma once

#include <ceres/ceres.h>

#include "camera_intrinsics.h"

namespace tl {

// Brief:
// FisheyeCameraModel
//
// Explanation:
//
// Reference:
class FisheyeCameraModel final : public CameraIntrinsics_<FisheyeCameraModel, 9>
{
    using Parent = CameraIntrinsics_<FisheyeCameraModel, 9>;

public:
    FisheyeCameraModel();

    inline static constexpr auto kType = CameraIntrinsicsType::Fisheye;

    void setFromMetaData(const CameraMetaData& meta) override;
    CameraMetaData toMetaData() const override;

    // ----------------------- Parameters Access ----------------------
    //
    enum IntrinsicsIndex
    {
        Skew = ExtraIndex,
        // K0 = 1,
        K1,
        K2,
        K3,
        K4,

        IntrinsicsSize,
    };
    static_assert(kNumParameters == IntrinsicsSize);

    void setSkew(double skew);
    double skew() const;

    void setRadialDistortion(double k1, double k2, double k3, double k4);
    double radialDistortion1() const;
    double radialDistortion2() const;
    double radialDistortion3() const;
    double radialDistortion4() const;
    inline auto k1() const { return radialDistortion1(); }
    inline auto k2() const { return radialDistortion2(); }
    inline auto k3() const { return radialDistortion3(); }
    inline auto k4() const { return radialDistortion4(); }

    std::vector<int> fixedParameterIndices(
        OptimizeIntrinsicsType flags) const override;

    Eigen::Matrix3d calibrationMatrix() const override;

    // --------------------- Point Mapping --------------------------------
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
    template <typename T>
    static void pixelToRayByIteration(const T* params, const T* pixel, T* ray);

    // NOTE: Learn from CamOdoCal, return 3D point
    // WARNING: Not fully tested
    template <typename T>
    static void undistortBackproject(const T* params, const T* distort,
                                     T* point3);

    template <typename T>
    static T calcDistortion(const T* params, const T* undistort);
};

/// -------------------------- Implementation -------------------------------
///
template <typename T>
bool FisheyeCameraModel::spaceToPixel(const T* intrinsics, const T* pt, T* px)
{
    // Normalize to plane at 1
    // T pt_norm[2];
    // pt_norm[0] = pt[0] / pt[2];
    // pt_norm[1] = pt[1] / pt[2];
    // pt_norm[2] = T(1);

    // Apply radial distortion.
    // NOTE: We pass in the entire 3D point instead of the projection onto a
    // plane or sphere.
    T px_d[2];
    distortPoint(intrinsics, pt, px_d);

    // Apply calibration parameters to transform normalized units into pixels.
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    px[0] = fx * px_d[0] + skew * px_d[1] + cx;
    px[1] = fy * px_d[1] + cy;

    return true;
}

template <typename T>
bool FisheyeCameraModel::pixelToSpace(const T* intrinsics, const T* px, T* pt)
{
    const T& fx = intrinsics[Fx];
    const T& y_x = intrinsics[YX];
    const T fy = fx * y_x;
    const T& cx = intrinsics[Cx];
    const T& cy = intrinsics[Cy];
    const T& skew = intrinsics[Skew];

    // Normalize the y coordinate first.
    T px_d[2];
    px_d[1] = (px[1] - cy) / fy;
    px_d[0] = (px[0] - cx - px_d[1] * skew) / fx;

    // Undo the radial distortion.
    enum
    {
        Iteration1,
        Iteration2,
        Backprojection
    };
    constexpr auto mode = Iteration2;

    if constexpr (mode == Iteration1) {
        pixelToRayByIteration(intrinsics, px, pt);
        return true;
    }
    if constexpr (mode == Iteration2) {
        undistortPoint(intrinsics, px_d, pt);
        pt[2] = T(1);

        return true;
    }
    if constexpr (mode == Backprojection) {
        FisheyeCameraModel::undistortBackproject(intrinsics, px_d, pt);
        return true;
    }

    return false;
}

template <typename T>
void FisheyeCameraModel::pixelToRayByIteration(const T* params, const T* px,
                                               T* ray)
{
    const T& fx = params[Fx];
    const T& y_x = params[YX];
    const T fy = fx * y_x;
    const T& cx = params[Cx];
    const T& cy = params[Cy];
    const T& skew = params[Skew];
    const T& k1 = params[K1];
    const T& k2 = params[K2];
    const T& k3 = params[K3];
    const T& k4 = params[K4];

    T px_norm[2];
    px_norm[1] = (px[1] - cy) / fy;
    px_norm[0] = (px[0] - cx - px_norm[1] * skew) / fx;

    const auto thetad = sqrt(px_norm[0] * px_norm[0] + px_norm[1] * px_norm[1]);

    // Empirical  value
    constexpr int kIteration{14};
    auto theta = thetad;
    T theta2, theta4, theta8;
    for (int i{0}; i < kIteration; ++i) {
        theta2 = theta * theta;
        theta4 = theta2 * theta2;
        theta8 = theta4 * theta4;
        // clang-format off
        theta = thetad / (T(1)
                          + k1 * theta2
                          + k2 * theta4
                          + k3 * theta4 * theta2
                          + k4 * theta8);
        // clang-format on
    }
    const auto scale = tan(theta) / thetad;

    ray[0] = scale * px_norm[0];
    ray[1] = scale * px_norm[1];
    ray[2] = T(1);
}

// For fisheye distortion, we use the angle of the undistorted point with
// respect to the optical axis to measure the magnitude of distortion. To make
// sure that the computation is stable, we pass in the 3-dimensional undistorted
// point and compute angle theta using the more stable atan2 so that we do not
// have to perform the (potentially unstable) perspective divide by z.
template <typename T>
bool FisheyeCameraModel::distortPoint(const T* intrinsics, const T* pt_u,
                                      T* pt_d)
{
    const auto scale = calcDistortion(intrinsics, pt_u);

    pt_d[0] = scale * pt_u[0];
    pt_d[1] = scale * pt_u[1];

    if (pt_u[2] < T(0)) {
        pt_d[0] = -pt_d[0];
        pt_d[1] = -pt_d[1];
    }

    return true;
}

template <typename T>
bool FisheyeCameraModel::undistortPoint(const T* intrinsics, const T* pt_d,
                                        T* pt_u)
{
    constexpr int kIterations{100};
    const T kUndistortionEpsilon = T(1e-10);

    T prev_pt_u[2];
    pt_u[0] = pt_d[0];
    pt_u[1] = pt_d[1];
    for (int i{0}; i < kIterations; ++i) {
        prev_pt_u[0] = pt_u[0];
        prev_pt_u[1] = pt_u[1];

        const T pt_u_norm[3]{pt_u[0], pt_u[1], T(1)};
        const auto scale =
            FisheyeCameraModel::calcDistortion(intrinsics, pt_u_norm);

        // We know that
        //      distorted point = theta_d / r * undistorted point,
        // so we can inverse the equation to estimate undistorted point.
        pt_u[0] = 1. / scale * pt_d[0];
        pt_u[1] = 1. / scale * pt_d[1];

        if (ceres::abs(pt_u[0] - prev_pt_u[0]) < kUndistortionEpsilon &&
            ceres::abs(pt_u[1] - prev_pt_u[1]) < kUndistortionEpsilon) {
            break;
        }
    }
    return true;
}

template <typename T>
void FisheyeCameraModel::undistortBackproject(const T* intrinsics,
                                              const T* pt_d, T* pt)
{
    const T& k1 = intrinsics[K1];
    const T& k2 = intrinsics[K2];
    const T& k3 = intrinsics[K3];
    const T& k4 = intrinsics[K4];

    const auto pt_d_norm = ceres::sqrt(pt_d[0] * pt_d[0] + pt_d[1] * pt_d[1]);

    static const T kVerySmallNumber = T(1e-10);
    const auto phi =
        pt_d_norm < kVerySmallNumber ? T(0) : ceres::atan2(pt_d[1], pt_d[0]);

    // FIXME: The logic here is actually a nested if condition
    int npow = 9;
    if (k4 == 0.0) {
        npow -= 2;
    }
    if (k3 == 0.0) {
        npow -= 2;
    }
    if (k2 == 0.0) {
        npow -= 2;
    }
    if (k1 == 0.0) {
        npow -= 2;
    }

    Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(npow + 1, 1);
    coeffs(0) = -pt_d_norm;
    coeffs(1) = 1.;

    if (npow >= 3) {
        coeffs(3) = k1;
    }
    if (npow >= 5) {
        coeffs(5) = k2;
    }
    if (npow >= 7) {
        coeffs(7) = k3;
    }
    if (npow >= 9) {
        coeffs(9) = k4;
    }

    auto theta = pt_d_norm;
    if (npow != 1) {
        // Get eigenvalues of companion matrix corresponding to polynomial.
        // Eigenvalues correspond to roots of polynomial.
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(npow, npow);
        A.block(1, 0, npow - 1, npow - 1).setIdentity();
        A.col(npow - 1) = -coeffs.block(0, 0, npow, 1) / coeffs(npow);

        Eigen::EigenSolver<Eigen::MatrixXd> es(A);
        Eigen::MatrixXcd eigval = es.eigenvalues();

        constexpr double kTolerance = 1e-10;

        std::vector<double> thetas;
        for (int i = 0; i < eigval.rows(); ++i) {
            if (std::abs(eigval(i).imag()) > kTolerance) {
                continue;
            }

            double t = eigval(i).real();
            if (t < -kTolerance) {
                continue;
            }

            thetas.push_back(std::max(t, 0.));
        }

        if (thetas.empty()) {
            theta = pt_d_norm;
        }
        else {
            theta = *std::min_element(thetas.begin(), thetas.end());
        }
    }

    pt[0] = std::sin(theta) * std::cos(phi);
    pt[1] = std::sin(theta) * std::sin(phi);
    pt[2] = std::cos(theta);
}

template <typename T>
T FisheyeCameraModel::calcDistortion(const T* intrinsics, const T* pt_u)
{
    const auto& k1 = intrinsics[K1];
    const auto& k2 = intrinsics[K2];
    const auto& k3 = intrinsics[K3];
    const auto& k4 = intrinsics[K4];

    const auto rr = pt_u[0] * pt_u[0] + pt_u[1] * pt_u[1];
    const auto r = sqrt(rr);

    // If the radius of the undistorted is too small then the divide by r below
    // is unstable. In this case, the point is very close to the center of
    // distortion and so we can assume there is no distortion.
    static const T kVerySmallNumber = T(1e-8);
    if (r < kVerySmallNumber) {
        return T(1);
    }

    // Using atan2 should be more stable than dividing by the denominator for
    // small z-values (i.e. viewing angles approaching 180 deg FOV).
    const auto theta = atan2(r, abs(pt_u[2]));
    const auto theta_2 = theta * theta;
    const auto theta_4 = theta_2 * theta_2;
    const auto theta_8 = theta_4 * theta_4;
    // clang-format off
    const auto thetad = theta * (T(1)
                                 + k1 * theta_2
                                 + k2 * theta_4
                                 + k3 * theta_2 * theta_4
                                 + k4 * theta_8);
    // clang-format on

    return thetad / r;
}

} // namespace tl
