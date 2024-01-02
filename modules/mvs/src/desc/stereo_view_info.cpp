#include "stereo_view_info.h"

#include <algorithm>

namespace thoht {

void TwoViewInfo::SwapCameras()
{
    CHECK_NE(focal_length1, 0.);
    CHECK_NE(focal_length2, 0.);

    // Swap focal length
    std::swap(focal_length1, focal_length2);

    // Invert the translation.
    Eigen::Vector3d neg_of_new_position;
    ceres::AngleAxisRotatePoint(rotation.data(), position.data(),
                                neg_of_new_position.data());
    position = -neg_of_new_position;
    rotation *= -1.0;
}

} // namespace thoht
