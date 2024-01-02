#pragma once

#include <opencv2/core/mat.hpp>

namespace thoht {

// A buffer and custom allocation mechanism for OpenCV Mats.
// 1. Lazy initialization to automatically determine the capacity
// 2. If the cacacity is exceeded, allocates more memory and re-uses that
// 3. Automatic reference counting (using OpenCV Mat internal refcounter) to
// check which slots are free
class FrameBuffer
{
public:
    explicit FrameBuffer(size_t capacityIncrease = 4,
                         size_t maxCapacity = 4 * 5);

    // Return a unique pointer to an free slot. If FrameBuffer is full, pointer
    // is empty.
    std::unique_ptr<cv::Mat> next(int rows, int cols, int type);

private:
    bool increaseCapacity(int rows, int cols, int type);

private:
    const size_t capacity_inc_;
    const size_t max_capacity_;

    std::vector<cv::Mat> buf_;
    size_t counter_{0};
};

} // namespace thoht
