#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {
namespace soc {

// NOTE: The data should be const, however, "const T*" cant be used to
// construct a cv::Mat
size_t parseImageData(uint8_t* data, cv::OutputArray dst);

} // namespace soc
} // namespace tl
