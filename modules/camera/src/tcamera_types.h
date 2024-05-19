#pragma once

#include <tCore/EnumUtils>

namespace tl {

// Explanation:
// The camera intrinsics parameters are defined by:
//   - Focal length
//   - Aspect ratio (y/x)
//   - Principal points (x, y)
//   - Skew
//   - Radial distortion
//   - Tangential distortion
// These intrinsic parameters may or may not be present for a given camera model
// and only the relevant intrinsics will be optimized per camera.
// It's common to assume that skew is 0, or aspect ratio is 1, and so we do not
// always desire to optimize all camera intrinsics. In many cases, the focal
// length is the only parameter we care to optimize.

// Users can specify which intrinsics to optimize by using a bitmask. For
// instance, FocalLength|PrincipalPoint will optimize the focal length and
// principal points.
enum class OptimizeIntrinsicsType
{
    None = 0x00,
    // Common
    FocalLength = 0x01,
    AspectRatio = 0x02,
    PrincipalPoint = 0x04,
    // Specialized
    Skew = 0x08,
    RadialDistortion = 0x10,
    TangentialDistortion = 0x20,

    All = FocalLength | AspectRatio | PrincipalPoint | Skew | RadialDistortion |
          TangentialDistortion,
    Common = FocalLength | AspectRatio | PrincipalPoint,
};
MAKE_FLAGS(OptimizeIntrinsicsType)

enum
{
    CameraLeft,
    CameraRight,

    StereoSize,
};

// Alternatives
inline constexpr int kCameraLeftId = CameraLeft;
inline constexpr int kCameraRightId = CameraRight;

} // namespace tl
