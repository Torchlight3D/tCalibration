#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>

namespace thoht {

class RectifierBase
{
public:
    using Ptr = std::shared_ptr<RectifierBase>;

    RectifierBase();
    virtual ~RectifierBase();

    static Ptr create();

    virtual void undistortImage(const cv::Mat& image, cv::Mat& out) const = 0;

protected:
};

} // namespace thoht
