#pragma once

#include "Quad.h"

namespace tl {
namespace stag {

struct Marker : Quad
{
    int id;
    cv::Mat C;

    Marker(const Quad &q, int inId);
    Marker(const Marker &other);

    void shiftCorners2(int shift);
    bool isSimilarIn(const std::vector<Marker> &markers) const;
};

} // namespace stag
} // namespace tl
