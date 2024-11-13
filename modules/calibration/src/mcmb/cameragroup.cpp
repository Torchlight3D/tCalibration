#include "cameragroup.h"

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "board.h"
#include "boardobs.h"
#include "boardgroup.h"
#include "boardgroupobs.h"
#include "camera.h"
#include "cameragroupobs.h"
#include "factors.h"
#include "frame.h"
#include "utils.h"

namespace tl::mcmb {

CameraGroup::CameraGroup(int refCamId, int camGroupId)
    : _refCamId(refCamId), _id(camGroupId)
{
}

void CameraGroup::insertCamera(std::shared_ptr<Camera> camera)
{
    _cameras[camera->_id] = camera;
    _camIds.push_back(camera->_id);
}

void CameraGroup::insertBoardGroupObservation(
    std::shared_ptr<BoardGroupObs> obObs)
{
    _boardGroupObservations[_boardGroupObservations.size()] = obObs;
}

void CameraGroup::insertFrame(std::shared_ptr<Frame> frame)
{
    _frames[frame->_id] = frame;
}

void CameraGroup::setCameraPose(cv::InputArray pose, int camId)
{
    cv::Mat rvec, tvec;
    Proj2RT(pose, rvec, tvec);
    setCameraPose(rvec, tvec, camId);
}

void CameraGroup::setCameraPose(cv::Mat rvec, cv::Mat tvec, int camId)
{
    _camPoses[camId] = {rvec.at<double>(0), rvec.at<double>(1),
                        rvec.at<double>(2), tvec.at<double>(0),
                        tvec.at<double>(1), tvec.at<double>(2)};
}

void CameraGroup::getCameraPose(cv::OutputArray rvec, cv::OutputArray tvec,
                                int camId) const
{
    const auto &pose = _camPoses.at(camId);

    cv::Mat{cv::Vec3d{pose[0], pose[1], pose[2]}}.reshape(1, 3).copyTo(rvec);
    cv::Mat{cv::Vec3d{pose[3], pose[4], pose[5]}}.reshape(1, 3).copyTo(tvec);
}

cv::Mat CameraGroup::cameraPose(int camId) const
{
    cv::Mat rvec, tvec;
    getCameraPose(rvec, tvec, camId);
    return RVecT2Proj(rvec, tvec);
}

void CameraGroup::computeObjPoseInCameraGroup()
{
    for (const auto &[_0, _frame] : _frames) {
        auto frame = _frame.lock();
        if (!frame) {
            continue;
        }

        for (const auto &[_1, _camGroupObs] : frame->_camGroupObservations) {
            auto camGroupObs = _camGroupObs.lock();
            if (!camGroupObs || _id != camGroupObs->_camGroupId) {
                continue;
            }

            for (const auto &[_2, _boardGroupObs] :
                 camGroupObs->_boardGroupObservations) {
                auto boardGroupObs = _boardGroupObs.lock();
                if (!boardGroupObs) {
                    continue;
                }

                boardGroupObs->setPoseInGroup(
                    cameraPose(boardGroupObs->_cameraId).inv() *
                    boardGroupObs->pose());
            }
        }
    }
}

void CameraGroup::refineCameraGroup(int iterations)
{
    LOG(INFO) << "Number of frames for camera group optimization: "
              << _frames.size();

    ceres::Problem problem;
    for (const auto &[_0, _frame] : _frames) {
        auto frame = _frame.lock();
        if (!frame) {
            continue;
        }

        for (const auto &[_1, _camGroupObs] : frame->_camGroupObservations) {
            auto camGroupObs = _camGroupObs.lock();
            if (!camGroupObs || _id != camGroupObs->_camGroupId) {
                continue;
            }

            for (const auto &[_2, _boardGroupObs] :
                 camGroupObs->_boardGroupObservations) {
                auto boardGroupObs = _boardGroupObs.lock();
                if (!boardGroupObs) {
                    continue;
                }

                auto boardGroup = boardGroupObs->_boardGroup.lock();
                auto cam = boardGroupObs->_camera.lock();
                if (!boardGroup || !cam) {
                    continue;
                }

                const auto &camId = boardGroupObs->_cameraId;
                const auto &objPoints = boardGroup->_points;
                const auto &cornerIds = boardGroupObs->_pointInds;
                const auto &imgPoints = boardGroupObs->_pixels;
                const auto &fx = cam->intrinsics_[0];
                const auto &fy = cam->intrinsics_[1];
                const auto &u0 = cam->intrinsics_[2];
                const auto &v0 = cam->intrinsics_[3];
                const auto &r1 = cam->intrinsics_[4];
                const auto &r2 = cam->intrinsics_[5];
                const auto &t1 = cam->intrinsics_[6];
                const auto &t2 = cam->intrinsics_[7];
                const auto &r3 = cam->intrinsics_[8];
                const bool refineCam = _refCamId != cam->_id;

                for (size_t i{0}; i < cornerIds.size(); i++) {
                    const auto &objPoint = objPoints[cornerIds[i]];
                    const auto &imgPoint = imgPoints[i];
                    auto cost = ReprojectionError_CameraGroupRef::create(
                        double(imgPoint.x), double(imgPoint.y),
                        double(objPoint.x), double(objPoint.y),
                        double(objPoint.z), fx, fy, u0, v0, r1, r2, r3, t1, t2,
                        refineCam, cam->_type);
                    problem.AddResidualBlock(
                        cost, new ceres::HuberLoss(1.), _camPoses[camId].data(),
                        camGroupObs->object_pose_[boardGroupObs->_boardGroupId]
                            .data());
                }
            }
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = iterations;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (const auto &[camId, _] : _camPoses) {
        LOG(INFO) << "Camera  " << camId << "  :: " << cameraPose(camId);
    }
}

void CameraGroup::refineCameraGroupAndBoardGroups(int iterations)
{
    LOG(INFO) << "Number of frames for camera group optimization: "
              << _frames.size();

    ceres::Problem problem;
    for (const auto &[_0, _frame] : _frames) {
        auto frame = _frame.lock();
        if (!frame) {
            continue;
        }

        for (const auto &[_1, _camGroupObs] : frame->_camGroupObservations) {
            auto camGroupObs = _camGroupObs.lock();
            if (!camGroupObs || _id != camGroupObs->_camGroupId) {
                continue;
            }

            for (const auto &[_2, _boardGroupObs] :
                 camGroupObs->_boardGroupObservations) {
                auto boardGroupObs = _boardGroupObs.lock();
                if (!boardGroupObs) {
                    continue;
                }

                auto boardGroup = boardGroupObs->_boardGroup.lock();
                auto cam = boardGroupObs->_camera.lock();
                if (!boardGroup || !cam) {
                    continue;
                }

                const auto &camId = boardGroupObs->_cameraId;
                const auto &cornerIds = boardGroupObs->_pointInds;
                const auto &imgPoints = boardGroupObs->_pixels;

                const auto &fx = cam->intrinsics_[0];
                const auto &fy = cam->intrinsics_[1];
                const auto &u0 = cam->intrinsics_[2];
                const auto &v0 = cam->intrinsics_[3];
                const auto &r1 = cam->intrinsics_[4];
                const auto &r2 = cam->intrinsics_[5];
                const auto &t1 = cam->intrinsics_[6];
                const auto &t2 = cam->intrinsics_[7];
                const auto &r3 = cam->intrinsics_[8];
                const bool refineCam = _refCamId != cam->_id;

                for (size_t i{0}; i < cornerIds.size(); i++) {
                    const auto &[boardId, cornerId] =
                        boardGroup->_boardIdCornerId[cornerIds[i]];

                    auto board = boardGroup->_boards[boardId].lock();
                    if (!board) {
                        continue;
                    }

                    const auto &imgPoint = imgPoints[i];
                    const auto &objPoint = board->point(cornerId);
                    const bool refineBoard = boardGroup->_refBoardId != boardId;

                    // key(boardid//ptsid)-->pts_ind_board
                    auto cost =
                        ReprojectionError_CameraGroupAndObjectRef::create(
                            double(imgPoint.x), double(imgPoint.y),
                            double(objPoint.x), double(objPoint.y),
                            double(objPoint.z), fx, fy, u0, v0, r1, r2, r3, t1,
                            t2, refineCam, refineBoard, cam->_type);
                    problem.AddResidualBlock(
                        cost, new ceres::HuberLoss(1.), _camPoses[camId].data(),
                        camGroupObs->object_pose_[boardGroupObs->_boardGroupId]
                            .data(),
                        boardGroup->_relBoardPoses[boardId].data());
                }
            }
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = iterations;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (const auto &[camId, _] : _camPoses) {
        LOG(INFO) << "Camera " << camId << ": " << cameraPose(camId);
    }
}

void CameraGroup::refineCameraGroupAndBoardGroupsAndIntrinsics(int iterations)
{
    LOG(INFO) << "Number of frames for camera group optimization: "
              << _frames.size();

    ceres::Problem problem;
    for (const auto &[_0, _frame] : _frames) {
        auto frame = _frame.lock();
        if (!frame) {
            continue;
        }

        for (const auto &[_1, _camGroupObs] : frame->_camGroupObservations) {
            auto camGroupObs = _camGroupObs.lock();
            if (!camGroupObs || _id != camGroupObs->_camGroupId) {
                continue;
            }

            for (const auto &[_2, _boardGroupObs] :
                 camGroupObs->_boardGroupObservations) {
                auto boardGroupObs = _boardGroupObs.lock();
                if (!boardGroupObs) {
                    continue;
                }

                auto boardGroup = boardGroupObs->_boardGroup.lock();
                auto cam = boardGroupObs->_camera.lock();
                if (!boardGroup || !cam) {
                    continue;
                }

                const auto &camId = boardGroupObs->_cameraId;
                const auto &cornerIds = boardGroupObs->_pointInds;
                const auto &corners = boardGroupObs->_pixels;
                const bool refineCam = _refCamId != cam->_id;

                for (size_t i = 0; i < cornerIds.size(); i++) {
                    const auto &[boardId, cornerId] =
                        boardGroup->_boardIdCornerId[cornerIds[i]];

                    auto board = boardGroup->_boards[boardId].lock();
                    if (!board) {
                        continue;
                    }

                    const auto &imgPoint = corners[i];
                    const auto &objPoint = board->point(cornerId);
                    const bool refineBoard = boardGroup->_refBoardId != boardId;

                    auto cost =
                        ReprojectionError_CameraGroupAndObjectRefAndIntrinsics::
                            create(double(imgPoint.x), double(imgPoint.y),
                                   double(objPoint.x), double(objPoint.y),
                                   double(objPoint.z), refineCam, refineBoard,
                                   cam->_type);
                    problem.AddResidualBlock(
                        cost, new ceres::HuberLoss(1.), _camPoses[camId].data(),
                        camGroupObs->object_pose_[boardGroupObs->_boardGroupId]
                            .data(),
                        boardGroup->_relBoardPoses[boardId].data(),
                        cam->intrinsics_.data());
                }
            }
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = iterations;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (const auto &[camId, _] : _camPoses) {
        LOG(INFO) << "Camera " << camId << ": " << cameraPose(camId);
    }
}

void CameraGroup::calcRpeMean() const
{
    for (const auto &[_0, _frame] : _frames) {
        auto frame = _frame.lock();
        if (!frame) {
            continue;
        }

        for (const auto &[_1, _camGroupObs] : frame->_camGroupObservations) {
            auto camGroupObs = _camGroupObs.lock();
            if (!camGroupObs || _id != camGroupObs->_camGroupId) {
                continue;
            }

            for (const auto &[_2, _boardGroupObs] :
                 camGroupObs->_boardGroupObservations) {
                auto boardGroupObs = _boardGroupObs.lock();
                if (!boardGroupObs) {
                    continue;
                }

                auto boardGroup = boardGroupObs->_boardGroup.lock();
                auto cam = boardGroupObs->_camera.lock();
                if (!boardGroup || !cam) {
                    continue;
                }

                const auto &camId = boardGroupObs->_cameraId;
                const auto &cornerIds = boardGroupObs->_pointInds;
                const auto &imgPoints = boardGroupObs->_pixels;

                std::vector<cv::Point3f> objPoints;
                objPoints.reserve(cornerIds.size());
                for (const auto &cornerId : cornerIds) {
                    objPoints.push_back(boardGroup->_points[cornerId]);
                }

                const auto transformedPoints = transformPoints(
                    objPoints,
                    camGroupObs->objectOrientation(
                        boardGroupObs->_boardGroupId),
                    camGroupObs->objectPosition(boardGroupObs->_boardGroupId));

                std::vector<cv::Point2f> projectedPoints;
                projectPointsWithDistortion(
                    transformedPoints, cameraOrientation(camId),
                    cameraPosition(camId), cam->cameraMatrix(),
                    cam->distortion(), projectedPoints, cam->_type);

                auto rpe{0.f};
                for (size_t i{0}; i < projectedPoints.size(); i++) {
                    rpe += cv::norm(imgPoints[i] - projectedPoints[i]);
                }
                rpe /= projectedPoints.size();

                // LOG(INFO) << "Camera " << current_cam_id
                //           << " detects " <<
                //           repro_pts.size()
                //           << " points of object "
                //           << it_obj3d_ptr->object_3d_id_
                //           << " with RPE: " << mean_error;
            }
        }
    }
}

} // namespace tl::mcmb
