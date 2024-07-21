#include "viewpairinfo.h"

#include <ceres/rotation.h>
#include <glog/logging.h>

#include <tCamera/Camera>
#include <tMath/Eigen/Rotation>

namespace tl {

using Eigen::Vector3d;

ViewPairInfo ViewPairInfo::fromCameraPair(const Camera& camera1,
                                          const Camera& camera2, bool normalize)
{
    const Vector3d rotation1 = camera1.orientationAsAngleAxis();
    const Vector3d rotation2 = camera2.orientationAsAngleAxis();

    ViewPairInfo info;
    info.focalLength1 = camera1.focalLength();
    info.focalLength2 = camera2.focalLength();
    info.rotation = MultiplyRotations(rotation2, -rotation1);

    // Compute the position of camera 2 in the coordinate system of camera 1
    // using the standard projection equation:
    //    X' = R * (X - c)
    // which yields:
    //    c2' = R1 * (c2 - c1).
    const Vector3d offset = camera2.position() - camera1.position();
    ceres::AngleAxisRotatePoint(rotation1.data(), offset.data(),
                                info.position.data());

    // Scale the relative position to be a unit-length vector.
    if (normalize) {
        info.position.normalize();
    }

    return info;
}

void ViewPairInfo::swapView()
{
    CHECK_NE(focalLength1, 0.);
    CHECK_NE(focalLength2, 0.);

    // Swap focal length
    std::swap(focalLength1, focalLength2);

    // Invert the translation.
    Vector3d newPosition;
    ceres::AngleAxisRotatePoint(rotation.data(), position.data(),
                                newPosition.data());
    position = -newPosition;
    rotation *= -1.;
}

} // namespace tl
