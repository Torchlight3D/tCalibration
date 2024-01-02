#pragma once

#include <opencv2/core/mat.hpp>

#include <AxCamera/Camera>

namespace thoht {

class OmnidirectionalCameraModel;

/// OpenCV implementation
bool initUndistortRectifyMap(const Camera& left, const Camera& right,
                             const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                             cv::OutputArray map1_left,
                             cv::OutputArray map2_left,
                             cv::OutputArray map1_right,
                             cv::OutputArray map2_right);

namespace io {
void toLubaStereoUndistortRectifyMapString(cv::InputArray map1_left,
                                           cv::InputArray map2_left,
                                           cv::InputArray map1_right,
                                           cv::InputArray map2_right,
                                           std::string& left_str,
                                           std::string& right_str);
}

} // namespace thoht
