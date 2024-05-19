#pragma once

#include <string>

namespace tl {

// TODO:
// 1. Setter/Getter are neither intuitive nor safe
template <int Size_t>
struct Elem
{
    double value[Size_t] = {0.0};
    bool is_set{false};
};

struct CameraMetaData
{
    int image_width{0};
    int image_height{0};

    std::string camera_intrinsics_model_type{"Pinhole"};

    // Camera intrinsics parameters.
    Elem<1> focal_length;
    Elem<2> principal_point;
    Elem<1> aspect_ratio;
    Elem<1> skew;

    // Up to 4 radial distortion parameters.
    // For fisheye cameras, the fisheye distortion parameters would be set as
    // radial_distortion.
    Elem<4> radial_distortion;
    Elem<2> tangential_distortion;

    // Extrinsics that may be available from EXIF or elsewhere.
    Elem<3> position;    // (x, y, z)
    Elem<3> orientation; // angle-axis

    // GPS priors. The altitude is measured as meters above sea level.
    Elem<1> latitude;  // meter
    Elem<1> longitude; // meter
    Elem<1> altitude;  // meter
};

} // namespace tl
