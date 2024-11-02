#pragma once

#include "TagDetection.h"
#include "TagFamily.h"

#include "../codec/TagCodes.h"

namespace AprilTags {

class TagDetector
{
public:
    explicit TagDetector(const tl::TagCodes& tagCodes, size_t blackBorder = 2)
        : thisTagFamily(tagCodes, blackBorder)
    {
    }

    std::vector<TagDetection> extractTags(const cv::Mat& image);

public:
    const TagFamily thisTagFamily;
};

} // namespace AprilTags
