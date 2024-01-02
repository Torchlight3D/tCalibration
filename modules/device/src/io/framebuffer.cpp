#include "framebuffer.h"

namespace thoht {

namespace {
constexpr size_t kDefaultCapacityInc{4};
}

FrameBuffer::FrameBuffer(size_t capacityIncrease, size_t maxCapacity)
    : capacity_inc_(capacityIncrease), max_capacity_(maxCapacity)
{
}

std::unique_ptr<cv::Mat> FrameBuffer::next(int rows, int cols, int type)
{
    if (buf_.empty()) {
        increaseCapacity(rows, cols, type);
    }

    for (size_t failures = 0; failures < buf_.size(); failures++) {
        auto &m = buf_[(counter_ = (counter_ + 1) % buf_.size())];
        assert(m.u && "u missing");
        // NOTE: This is not part of the OpenCV public API so use with care
        // This version should be thread-safe
        const auto refcount = CV_XADD(&m.u->refcount, 0);
        assert(refcount >= 1);
        if (refcount == 1) {
            return std::make_unique<cv::Mat>(m);
        }
    }

    if (increaseCapacity(rows, cols, type)) {
        return next(rows, cols, type);
    }

    return nullptr;
}

bool FrameBuffer::increaseCapacity(int rows, int cols, int type)
{
    auto size = buf_.size();
    if (size == max_capacity_) {
        return false;
    }

    size = std::min(size + capacity_inc_, max_capacity_);
    while (buf_.size() < size) {
        buf_.emplace_back(rows, cols, type);
    }
    return true;
}

} // namespace thoht
