#include "boardgroup.h"

#include "board.h"
#include "boardobs.h"
#include "boardgroupobs.h"
#include "camera.h"
#include "factors.h"
#include "frame.h"
#include "utils.h"

namespace tl::mcmb {

BoardGroup::BoardGroup(int refBoardId, int id)
    : _refBoardId(refBoardId), _id(id), _numPoints(0)
{
}

void BoardGroup::insertFrame(std::shared_ptr<Frame> frame)
{
    _frames[frame->_id] = frame;
}

void BoardGroup::insertBoard(std::shared_ptr<Board> board)
{
    _boards[board->_id] = board;
    _numPoints += board->numPoints();
}

void BoardGroup::insertObservation(std::shared_ptr<BoardGroupObs> obs)
{
    _observations[_observations.size()] = obs;
}

void BoardGroup::setBoardPose(cv::Mat pose, int boardId)
{
    cv::Mat rvec, tvec;
    Proj2RT(pose, rvec, tvec);
    setBoardPose(rvec, tvec, boardId);
}

void BoardGroup::setBoardPose(cv::Mat rvec, cv::Mat tvec, int boardId)
{
    _relBoardPoses[boardId] = {rvec.at<double>(0), rvec.at<double>(1),
                               rvec.at<double>(2), tvec.at<double>(0),
                               tvec.at<double>(1), tvec.at<double>(2)};
}

void BoardGroup::getBoardPose(cv::Mat &rvec, cv::Mat &tvec, int boardId) const
{
    const auto &pose = _relBoardPoses.at(boardId);

    cv::Mat rot_v = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat trans_v = cv::Mat::zeros(3, 1, CV_64F);
    rot_v.at<double>(0) = pose[0];
    rot_v.at<double>(1) = pose[1];
    rot_v.at<double>(2) = pose[2];
    trans_v.at<double>(0) = pose[3];
    trans_v.at<double>(1) = pose[4];
    trans_v.at<double>(2) = pose[5];
    rot_v.copyTo(rvec);
    trans_v.copyTo(tvec);
}

cv::Mat BoardGroup::boardPose(int boardId) const
{
    cv::Mat rvec, tvec;
    getBoardPose(rvec, tvec, boardId);
    return RVecT2Proj(rvec, tvec);
}

void BoardGroup::refineBoardGroup(int iterations)
{
    ceres::Problem problem;
    for (const auto &[_0, _boardGroupObs] : _observations) {
        auto boardGroupObs = _boardGroupObs.lock();
        if (!boardGroupObs) {
            continue;
        }

        for (const auto &[_1, _boardObs] : boardGroupObs->_boardObservations) {
            auto boardObs = _boardObs.lock();
            if (!boardObs || !boardObs->_valid) {
                continue;
            }

            auto board = boardObs->_board.lock();
            auto cam = boardObs->_camera.lock();
            if (!board || !cam) {
                continue;
            }

            const auto &boardId = boardObs->_boardId;
            const auto &objPoints = board->points();
            const auto &cornerIds = boardObs->_cornerIds;
            const auto &imgPoints = boardObs->_corners;

            const auto &fx = cam->intrinsics_[0];
            const auto &fy = cam->intrinsics_[1];
            const auto &u0 = cam->intrinsics_[2];
            const auto &v0 = cam->intrinsics_[3];
            const auto &r1 = cam->intrinsics_[4];
            const auto &r2 = cam->intrinsics_[5];
            const auto &t1 = cam->intrinsics_[6];
            const auto &t2 = cam->intrinsics_[7];
            const auto &r3 = cam->intrinsics_[8];
            const bool refineBoard = _refBoardId != boardId;

            for (size_t i{0}; i < cornerIds.size(); i++) {
                const auto &objPoint = objPoints[cornerIds[i]];
                const auto &imgPoint = imgPoints[i];
                auto *cost = ReprojectionError_3DObjRef::create(
                    double(imgPoint.x), double(imgPoint.y), double(objPoint.x),
                    double(objPoint.y), double(objPoint.z), fx, fy, u0, v0, r1,
                    r2, r3, t1, t2, refineBoard, cam->_type);
                problem.AddResidualBlock(cost, new ceres::HuberLoss(1.),
                                         boardGroupObs->_pose.data(),
                                         _relBoardPoses[boardId].data());
            }
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = iterations;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    updateObjectPoints();
}

void BoardGroup::updateObjectPoints()
{
    for (const auto &[_, _board] : _boards) {
        auto board = _board.lock();
        if (!board) {
            continue;
        }

        const auto &boardId = board->_id;

        const auto newPoints = transformPoints(
            board->points(), boardOrientation(boardId), boardPosition(boardId));

        // Replace the keypoints
        for (size_t i{0}; i < newPoints.size(); i++) {
            _points[_boardIdCornerIdToObjId[std::make_pair(boardId, i)]] =
                newPoints[i];
        }
    }
}

} // namespace tl::mcmb
