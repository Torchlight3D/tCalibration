#pragma once

#include "tag_detection.h"
#include "tag_family.h"

namespace cv {
class Mat;
}

namespace apriltags {

class TagDetector
{
public:
    explicit TagDetector(const TagCodes& tagCodes, size_t blackBorder = 2);

    std::vector<TagDetection> extractTags(const cv::Mat& image);
    std::vector<TagDetection> extractTagsKalibr(const cv::Mat& image);

public:
    const TagFamily _tagFamily;
};

} // namespace apriltags
