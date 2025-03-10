#pragma once

#include <opencv2/core/types.hpp>

#include "Homography33.h"

namespace AprilTags {

class FloatImage;
class Segment;

//! Represents four segments that form a loop, and might be a tag.
class Quad
{
public:
    //!< Minimum size of a tag (in pixels) as measured along edges and
    //!< diagonals
    inline static constexpr int minimumEdgeLength = 6;

    //!< Early pruning of quads with insane ratios.
    inline static constexpr float maxQuadAspectRatio = 32.f;

    //! Constructor
    /*! (x,y) are the optical center of the camera, which is
     *   needed to correctly compute the homography. */
    Quad(const std::vector<cv::Point2f>& p, const cv::Point2f& opticalCenter);

    // Interpolate given that the lower left corner of the lower left cell is at
    // (-1, -1) and the upper right corner of the upper right cell is at (1, 1).
    cv::Point2f interpolate(float x, float y) const;

    //! Same as interpolate, except that the coordinates are interpreted between
    //! 0 and 1, instead of -1 and 1.
    cv::Point2f interpolate01(float x, float y) const;

    //! Points for the quad (in pixel coordinates), in counter clockwise order.
    //! These points are the intersections of segments.
    std::vector<cv::Point2f> quadPoints;

    //! Segments composing this quad
    std::vector<Segment*> segments;

    //! Total length (in pixels) of the actual perimeter observed for the quad.
    /*! This is in contrast to the geometric perimeter, some of which
     *  may not have been directly observed but rather inferred by
     *  intersecting segments. Quads with more observed perimeter are
     *  preferred over others. */
    float observedPerimeter;

    //! Given that the whole quad spans from (0,0) to (1,1) in "quad space",
    //! compute the pixel coordinates for a given point within that quad.
    /*!  Note that for most of the Quad's existence, we will not know the
     * correct orientation of the tag. */
    Homography33 homography;

    //! Searches through a vector of Segments to form Quads.
    /*  @param quads any discovered quads will be added to this list
     *  @param path  the segments currently part of the search
     *  @param parent the first segment in the quad
     *  @param depth how deep in the search are we?
     */
    static void search(std::vector<Segment*>& path, Segment& parent, int depth,
                       std::vector<Quad>& quads,
                       const cv::Point2f& opticalCenter);

#ifdef INTERPOLATE
private:
    Eigen::Vector2f p0, p3, p01, p32;
#endif
};

} // namespace AprilTags
