#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace tl {

// A generic keypoint class that mimics a protocol buffer The only variable
// thats must be set are x, y, and the type. All other variable are optional and
// will be set to THEIA_INVALID_KEYPOINT_VAR by default.
#define THEIA_INVALID_KEYPOINT_VAR -9999

class Keypoint
{
public:
    enum KeypointType
    {
        INVALID = -1,
        OTHER = 0,
        SIFT = 1,
        AKAZE = 2,
    };

    Keypoint()
        : Keypoint(THEIA_INVALID_KEYPOINT_VAR, THEIA_INVALID_KEYPOINT_VAR,
                   Keypoint::INVALID)
    {
    }

    Keypoint(double x, double y, KeypointType type)
        : x_(x),
          y_(y),
          keypoint_type_(type),
          strength_(THEIA_INVALID_KEYPOINT_VAR),
          scale_(THEIA_INVALID_KEYPOINT_VAR),
          orientation_(THEIA_INVALID_KEYPOINT_VAR)
    {
    }

    ~Keypoint() {}

    // Keypoint type.
    inline KeypointType keypoint_type() const { return keypoint_type_; }
    inline void set_keypoint_type(KeypointType type) { keypoint_type_ = type; }

    inline double x() const { return x_; }
    inline void set_x(double x) { x_ = x; }
    inline double y() const { return y_; }
    inline void set_y(double y) { y_ = y; }

    // Optional variable strength.
    inline bool has_strength() const
    {
        return strength_ != THEIA_INVALID_KEYPOINT_VAR;
    }
    inline double strength() const { return strength_; }
    inline void set_strength(double strength) { strength_ = strength; }

    // Optional variable scale.
    inline bool has_scale() const
    {
        return scale_ != THEIA_INVALID_KEYPOINT_VAR;
    }
    inline void set_scale(double scale) { scale_ = scale; }
    inline double scale() const { return scale_; }

    // Optional variable orientation.
    inline bool has_orientation() const
    {
        return orientation_ != THEIA_INVALID_KEYPOINT_VAR;
    }
    inline double orientation() const { return orientation_; }
    inline void set_orientation(double orientation)
    {
        orientation_ = orientation;
    }

private:
    double x_, y_;
    KeypointType keypoint_type_;
    double strength_;
    double scale_;
    double orientation_;
};

struct KeypointsAndDescriptors
{
    std::string image_name;
    std::vector<Keypoint> keypoints;
    std::vector<Eigen::VectorXf> descriptors;
};

} // namespace tl
