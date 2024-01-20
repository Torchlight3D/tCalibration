#pragma once

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

namespace tl {

template <typename T>
inline Eigen::Vector2<T> cvPoint2ToEigen(const cv::Point_<T>& pt)
{
    return {pt.x, pt.y};
}

template <typename T>
inline Eigen::Vector3<T> cvPoint3ToEigen(const cv::Point3_<T>& pt)
{
    return {pt.x, pt.y, pt.z};
}

template <typename T>
inline cv::Point_<T> eigenToCvPoint2(const Eigen::Vector2<T> pt)
{
    return {pt.x(), pt.y()};
}

template <typename T>
inline cv::Point3_<T> eigenToCvPoint3(const Eigen::Vector3<T> pt)
{
    return {pt.x(), pt.y(), pt.z()};
}

} // namespace tl
