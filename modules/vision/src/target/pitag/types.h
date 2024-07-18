#pragma once

#include <opencv2/core/mat.hpp>

namespace ipa_Fiducials {

/// Enum to encode the different feature types
typedef enum
{
    TYPE_UNDEFINED = 0x00000000, ///< Undefined feature type
    TYPE_PI = 0x00000001, ///< PI fiducial from Bergamasco et al. 'Pi-Tag: a
                          ///< fast image-space marker design basedon projective
                          ///< invariants'
    TYPE_ARUCO = 0x00000002 ///< ArUco fiducial from
                            ///< http://www.uco.es/investiga/grupos/ava/node/26'
} t_FiducialType;

struct t_pose
{
    int id; ///< Unique ID of the marker

    cv::Mat rot;   ///< rodrigues rotation vector from tag coordinate system to
                   ///< camera coordinate system
    cv::Mat trans; ///< translation from tag coordinate system to camera
                   ///< coordinate system
};

struct t_points
{
    int id; ///< Unique ID of the marker

    std::vector<cv::Point2f>
        marker_points; ///< ellipse coordinates in marker coordinate system
    std::vector<cv::Point2f>
        image_points; ///< ellipse coordinates in marker coordinate system
};

} // namespace ipa_Fiducials
