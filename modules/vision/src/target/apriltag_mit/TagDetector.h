#pragma once

#include "TagDetection.h"
#include "TagFamily.h"

namespace AprilTags {

class TagDetector
{
public:
    const TagFamily thisTagFamily;

    //! Constructor
    // note: TagFamily is instantiated here from TagCodes
    explicit TagDetector(const TagCodes& tagCodes, size_t blackBorder = 2)
        : thisTagFamily(tagCodes, blackBorder)
    {
    }

    std::vector<TagDetection> extractTags(const cv::Mat& image);
};

} // namespace AprilTags
