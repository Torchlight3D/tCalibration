#pragma once

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include <tMath/Eigen/Types>
#include <tVision/Target/CalibBoardBase>

namespace tl {

// class CameraObservation
//{
// public:
//     explicit CameraObservation(CalibBoardBase::Ptr target,
//                                const cv::Mat &image = cv::Mat());

//    // properties
//    void setTarget(CalibBoardBase::Ptr target);
//    CalibBoardBase::ConstPtr target() const;

//    void setImage(const cv::Mat &image);
//    cv::Mat image() const;
//    void clearImage();

//    // data
//    void setImagePoints(const std::vector<cv::Point2d> &imagePoints,
//                        const std::vector<bool> &observed);
//    void updateImagePoint(int index, const cv::Point2d &point);
//    void removeImagePoint(int index);
//    bool observed(int index) const;
//    bool hasSuccessfulObservation() const;

//    int observedCornersInTarget(std::vector<cv::Point3d> &corners) const;
//    int observedCornersInImage(std::vector<cv::Point2d> &corners) const;
//    int observedCornerIndexes(std::vector<int> &cornerIndexes) const;

//    cv::Vec2d offset(int index) const;
//    double reprojectionError(int index) const;

//    //! camera pose in target (board) frame
//    void setObservePose(const Eigen::Isometry3d &transform);
//    const Eigen::Isometry3d &observePose() const;

//    // actions
//    int calcCornerReprojection(const CameraModelBase::Ptr camera,
//                               std::vector<cv::Point2d> &reprojectPoints)
//                               const;

// private:
//     void clearObservations();

// private:
//     cv::Mat m_image;              // TODO: consider own data here
//     CalibBoardBase::Ptr m_target; // not own, live longer than
//     CalibObservation std::vector<cv::Point2d> m_imgPoints;
//     std::vector<cv::Point3d> m_objPoints;
//     std::vector<cv::Vec2d> m_offsets;
//     std::vector<bool> m_observed;
//     Eigen::Isometry3d m_pose;
//     bool m_calibrated;
// };

// class CalibObservation
//{
// public:
//     explicit CalibObservation(int cameraCount);

// private:
//     std::vector<CameraObservation> m_obs;
// };

// class CalibData
//{
// public:
//     CalibData();

// private:
//     std::vector<CalibObservation> m_obs;
// };

struct Observation
{
    enum Status
    {
        None,
        Loaded,
        Failed,
        Passed
    };

    cv::Mat image;
    cv::Mat imageDebug;
    Vector2dList corners;
    std::vector<int> ids;
    std::string source;
    Status status{None};
    double timestamp; // sec
    int groupId;

    Observation() = default;
    Observation(const cv::Mat &img, double timestamp, int groupId = 0)
        : timestamp(timestamp), groupId(groupId)
    {
        // real data here
        img.copyTo(image);
        img.copyTo(imageDebug);
    }

    void updateDetection(const Vector2dList &corners,
                         const std::vector<int> &ids)
    {
        if (corners.size() != ids.size()) {
            return;
        }

        this->corners = corners;
        this->ids = ids;
        status = Passed;
    }

    size_t cornerCount() const
    {
        return (status == Passed) ? corners.size() : 0;
    }
};

using Observations = std::vector<Observation>;

} // namespace tl
