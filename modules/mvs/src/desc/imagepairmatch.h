#pragma once

#include "feature.h"
#include "viewpairinfo.h"

namespace tl {

struct ImagePairMatch
{
public:
    std::string image1;
    std::string image2;

    // If the matches are verified matches then the two view info contains the
    // relative pose information between the images.
    ViewPairInfo twoview_info;

    // Feature locations in pixel coordinates. If the match is a verified match
    // then this only contains inlier correspondences.
    std::vector<Feature2D2D> correspondences;
};

} // namespace tl
