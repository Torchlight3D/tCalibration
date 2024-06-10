#pragma once

#include <array>
#include <optional>
#include <string>

namespace tl {

struct CameraMetaData
{
    template <size_t Size_t, typename T = double>
    using Item_ = std::optional<std::array<T, Size_t>>;

    template <size_t Size_t>
    using Item = Item_<Size_t>;

    Item_<2, int> imageSize; // [width, height]

    // Intrinsics - Projection
    std::string intrinsicType{"Pinhole"};

    Item<1> focalLength;
    Item<1> aspectRatio;
    Item<2> principalPoint; // [cx, cy]
    Item<1> skew;

    // Intrinsics - Distortion
    Item<4> radialDistortion;
    Item<2> tangentialDistortion;

    // Extrinsics
    Item<3> position;    // meter
    Item<3> orientation; // Angle axis

    // GPS
    Item<1> latitude;  // meter
    Item<1> longitude; // meter
    Item<1> altitude;  // meter

    CameraMetaData() = default;

    /// Alias
    // NOTE: Check has_value() before calling
    const auto& imageWidth() const { return imageSize.value()[0]; }
    const auto& imageHeight() const { return imageSize.value()[1]; }

    const auto& f() const { return focalLength.value()[0]; }

    const auto& cx() const { return principalPoint.value()[0]; }
    const auto& cy() const { return principalPoint.value()[1]; }

    // At least we need to have image size
    bool isValid() const { return imageSize.has_value(); }

    // Focal length is the most essential parameter for any calibrated camera.
    bool calibrated() const { return isValid() && focalLength.has_value(); }
};

} // namespace tl
