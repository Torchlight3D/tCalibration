#pragma once

#include <functional>
#include <vector>

#include <tMotion/ImuData>

namespace tl {

class SplineErrorWeighting
{
public:
    // Brief:
    // Find knot spacing and variance from IMU readings
    //
    // Explanation:
    // This function first determine the maximum knot spacing of the input data
    // that fulfill the given quality. Then, it estimates the variance of the
    // spline error with this knot spacing.
    //
    // Input:
    //     data: imu data to fit
    //     quality: should be in range of [0., 1.], the better the larger.
    //     minSpacing: minimum acceptable knot spacing
    //     maxSpacing: maximum acceptable knot spacing
    // Output:
    //     spacing: the estimated knot spacing
    //     var: the spline fit error for input data and knot spacing
    void estKnotSpacingAndVariance(const ImuReadings& data, double quality,
                                   double minSpacing, double maxSpacing,
                                   double& spacing, double& var);
};

} // namespace tl
