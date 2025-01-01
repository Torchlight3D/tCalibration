#include "utils.h"

#include <opencv2/imgproc.hpp>

#include "types.h"

namespace tl {

namespace {

// TODO: Move to Vision module
template <typename T>
inline cv::Mat createMatFromBuffer(T* buffer, int rows, int cols, int channels,
                                   bool copy = false)
{
    const cv::Mat mat{
        rows, cols, CV_MAKE_TYPE(cv::traits::Type<T>::value, channels), buffer};
    if (copy) {
        return mat.clone();
    }
    return mat;
}

} // namespace

namespace soc {

size_t parseImageData(uint8_t* data, cv::OutputArray dst)
{
    const auto msgMeta = reinterpret_cast<const hb_vio_buffer_t*>(data);
    const auto& imgMeta = msgMeta->img_addr;
    const auto nv12 =
        createMatFromBuffer(data + sizeof(hb_vio_buffer_t),
                            imgMeta.height * 3 / 2, imgMeta.width, 1);
    cv::cvtColor(nv12, dst, cv::COLOR_YUV2BGR_NV12);

    // [Optional]
    auto size = sizeof(hb_vio_buffer_t);
    for (uint32_t i{0}; i < msgMeta->img_info.planeCount; ++i) {
        size += msgMeta->img_info.size[i];
    }
    return size;
};

} // namespace soc
} // namespace tl
