#include "calibration.h"

#include <filesystem>
#include <format>
#include <map>

#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <tCore/Math>
#include <tCore/ThreadPool>
#include <tVision/Target/AprilTagKalibr>
#include <tVision/Target/AprilTagOfficial>
#include <tVision/Serialization>

#include "board.h"
#include "boardobs.h"
#include "boardgroup.h"
#include "boardgroupobs.h"
#include "camera.h"
#include "cameraobs.h"
#include "cameragroup.h"
#include "cameragroupobs.h"
#include "frame.h"
#include "systemgraph.h"
#include "utils.h"

namespace fs = std::filesystem;

using Eigen::Vector3d;

namespace tl::mcmb {

namespace {

inline cv::Mat computeDistanceBetweenPoints(
    const std::vector<cv::Point2f> &points1,
    const std::vector<cv::Point2f> &points2)
{
    cv::Mat errors;
    for (size_t i{0}; i < points2.size(); i++) {
        errors.push_back(cv::norm(points1[i] - points2[i]));
    }
    return errors;
}

std::map<std::pair<int, int>, cv::Mat> calcAveragePosePairs(
    const std::map<std::pair<int, int>, std::vector<cv::Mat>> &posePairs,
    bool useQuaternionAverage)
{
    std::map<std::pair<int, int>, cv::Mat> avgPoses;
    for (const auto &[pairId, pairPoses] : posePairs) {
        const auto avgPose = calcAveragePose(pairPoses);
        avgPoses.insert({pairId, avgPose});

        LOG(INFO) << "Average pose: " << avgPose;
    }

    return avgPoses;
}

} // namespace

class TaskDir
{
public:
    TaskDir(const std::string &root) : _root(root) {}

    std::filesystem::path cameraDir() const
    {
        return fs::path{_root} / "Cameras";
    }
    std::filesystem::path sensorDir() const
    {
        return fs::path{_root} / "Sensors";
    }
    std::filesystem::path resultDir() const
    {
        return fs::path{_root} / "Results";
    }
    inline auto detectionDir() const { return resultDir() / "Detection"; }
    inline auto reprojectionDir() const { return resultDir() / "Reprojection"; }
    inline auto keyPointsFile() const
    {
        return (resultDir() / "detected_keypoints_data.yml").string();
    }
    inline auto calibratedCamerasFile() const
    {
        return (resultDir() / "calibrated_cameras_data.yml").string();
    }
    inline auto calibratedBoardsFile() const
    {
        return (resultDir() / "calibrated_objects_data.yml").string();
    }
    inline auto calibratedBoardPosesFile() const
    {
        return (resultDir() / "calibrated_objects_pose_data.yml").string();
    }
    inline auto reprojectionErrorFile() const
    {
        return (resultDir() / "reprojection_error_data.yml").string();
    }

private:
    std::string _root;
};

class Calibration::Impl
{
public:
    void insertBoardObservation(int camId, int frameId, int boardId,
                                const std::vector<cv::Point2f> &corners,
                                const std::vector<int> &cornerIds,
                                double timestamp, const std::string &filename);

    void calibrateCameras();
    void initializeCameraCalibration();
    void estimateCameraBoardRelativePose();
    void refineCameraCalibration();
    // NOTE: Log only
    void calcBoardObservationRpe() const;

    void calibrateBoardGroups();
    void calcBoardPairPoses();
    void initBoardGraph();
    void initBoardGroups();
    void initBoardGroupObservation(int boardGroupId);
    void initBoardGroupObservations();
    void estimateBoardGroupPoses();
    void refineBoardGroupPoses();
    // NOTE: Log only
    void calcBoardGroupObservationRpe() const;

    void calibrateCameraGroups();
    void calcCameraPairPoses();
    void initCameraGraph();
    void initCameraGroup();
    void initCameraGroupObservation(int camGroupId);
    void initCameraGroupObservations();
    void estimateBoardGroupInCameraGroupPoses();
    void refineAllCameraGroupAndObjects();

    void merge3DObjects();
    void findPairObjectForNonOverlap();
    // Initialize the pose between all non overlapping camera groups
    void findPoseNoOverlapAllCamGroup();
    // Initialize camera group graph without overlaping
    void initInterCamGroupGraph();
    void mergeCameraGroups();
    void mergeCameraGroupObservations();

    void calcCameraGroupsRpe() const;
    void refineAllCameraGroup();

    void insertBoardGroupObservation(
        std::shared_ptr<BoardGroupObs> observation);
    void initNonOverlapPair(int camGroupId1, int camGroupId2);
    void computeObjectsPairPose();
    void initInterObjectsGraph();
    void mergeBoardGroups();
    void mergeAllObjectObs();
    void refineAllCameraGroupAndObjectsAndIntrinsic();
    double calcOverallRpeMean() const;

    void saveDetectedKeypoints(const std::string &filename) const;
    void saveCameraParameters(const std::string &filename) const;
    void saveBoardGroups(const std::string &filename) const;
    void saveBoardGroupPoses(const std::string &filename) const;
    void saveReprojectionErrors(const std::string &filename) const;
    void saveDetection(const std::string &root) const;
    void saveReprojection(const std::string &root) const;

public:
    Options _opts;

    // hand-eye technique
    int he_approach_ = 0;

    // Data cache
    std::vector<std::shared_ptr<Board>> _boards;
    std::vector<std::shared_ptr<Camera>> _cameras;
    std::map<int, std::shared_ptr<Frame>> _frames;

    std::map<int, std::shared_ptr<BoardObs>> _boardObservations;
    // (Camera Id, Frame Id) -> Camera Observation
    std::map<std::pair<int, int>, std::shared_ptr<CameraObs>> _camObservations;

    std::map<int, std::shared_ptr<BoardGroup>> _boardGroups;
    std::map<int, std::shared_ptr<BoardGroupObs>> _boardGroupObservations;
    std::map<int, std::shared_ptr<CameraGroup>> _camGroups;
    // (CameraGroup Id, Frame Id) -> CameraGroup Observation
    std::map<std::pair<int, int>, std::shared_ptr<CameraGroupObs>>
        _camGroupObservations;

    // Relationship between boards seen in the same images
    // (Board Id1, Board Id2) -> Vector of poses
    std::map<std::pair<int, int>, std::vector<cv::Mat>> _boardPairPoses;
    // (Board Id1, Board Id2) -> Transform between two boards
    std::map<std::pair<int, int>, cv::Mat> _boardIdPairToTransform;
    // Inter-boards relationship. Vertex: Board Id, Edge: co-visibility count
    Graph _covisBoardGraph;

    // Relationship between cameras seeing the same BoardGroup
    // (Board Id1, Board Id2) -> Vector of poses (???)
    std::map<std::pair<int, int>, std::vector<cv::Mat>>
        _cameraIdPairToTransforms;
    // (Camera Id1, Camera Id2) -> Transform between two cameras
    std::map<std::pair<int, int>, cv::Mat> _cameraIdPairToTransform;
    // Inter-cameras relationship. Vertex: Camera Id, Edge: co-visibility count
    Graph _covisCameraGraph;

    // Relationship between BoardGroup seen in the same images
    // (BoardGroup Id1, BoardGroup Id2) -> Vector of poses
    std::map<std::pair<int, int>, std::vector<cv::Mat>>
        _boardGroupIdPairToTransforms;
    // (BoardGroup Id1, BoardGroup Id2) -> Transform between BoardGroups
    std::map<std::pair<int, int>, cv::Mat> _boardGroupIdPairsToTransform;
    // Inter-BoardGroup relationship.
    // Vertex: BoardGroup Id, Edge: co-visibility count
    Graph _covisBoardGroupGraph;

    // Non-overlaping items
    // (CamGroup Id1, CamGroup Id2) -> (BoardGroup Id1, BoardGroup Id2)
    std::map<std::pair<int, int>, std::pair<int, int>>
        _camGroupIdPairToTransforms;
    // (CamGroup Id1, CamGroup Id2) -> Transform between two camera groups
    std::map<std::pair<int, int>, cv::Mat> _camGroupIdPairToTransform;
    // (CamGroup Id1, CamGroup Id2) -> Common frames count between two groups
    std::map<std::pair<int, int>, int> _camGroupIdPairToNumFrames;
    // Inter-Camera Group pose determined without overlapping
    Graph _nonOverlapCamGroupGraph;
};

void Calibration::Impl::insertBoardObservation(
    int camId, int frameId, int boardId,
    const std::vector<cv::Point2f> &corners, const std::vector<int> &cornerIds,
    double timestamp, const std::string &filename)
{
    if (camId >= _cameras.size() || boardId >= _boards.size()) {
        LOG(WARNING) << "Invalid camera Id " << camId << " or board Id "
                     << boardId;
        return;
    }

    if (!_frames.contains(frameId)) {
        auto frame = std::make_shared<Frame>(frameId, timestamp);
        _frames.insert({frameId, frame});
    }

    const auto camObsId = std::make_pair(camId, frameId);
    if (!_camObservations.contains(camObsId)) {
        auto camObs = std::make_shared<CameraObs>(camObsId);
        _camObservations.insert({camObsId, camObs});
    }

    auto &camera = _cameras.at(camId);
    auto &board = _boards.at(boardId);
    auto &frame = _frames.at(frameId);
    auto &camObs = _camObservations.at(camObsId);

    camera->insertFrame(frame);
    board->insertFrame(frame);
    frame->insertObservation(camId, filename);
    frame->insertCameraObservation(camObs);

    auto boardObs = std::make_shared<BoardObs>(camId, frameId, boardId, corners,
                                               cornerIds, camera, board);
    _boardObservations[_boardObservations.size()] = boardObs;
    camera->insertBoardObservation(boardObs);
    board->insertObservation(boardObs);
    frame->insertBoardObservation(boardObs);
    camObs->insertBoardObservation(boardObs);
}

void Calibration::Impl::calibrateCameras()
{
    initializeCameraCalibration();
    estimateCameraBoardRelativePose();
    if (!_opts.fixIntrinsics) {
        refineCameraCalibration();
    }
    calcBoardObservationRpe();
}

void Calibration::Impl::initializeCameraCalibration()
{
    LOG(INFO) << "Initializing camera calibration...";

    for (const auto &camera : _cameras) {
        camera->initializeCalibration();
    }
}

void Calibration::Impl::estimateCameraBoardRelativePose()
{
    LOG(INFO) << "Estimating camera-board relative pose...";

    for (const auto &[_, boardObs] : _boardObservations) {
        boardObs->estimatePose(_opts.errorThreshold, _opts.numIterations);
    }
}

void Calibration::Impl::refineCameraCalibration()
{
    LOG(INFO) << "Refine camera calibration...";

    for (const auto &camera : _cameras) {
        camera->refineIntrinsicCalibration(_opts.numIterations);
    }
}

void Calibration::Impl::calcBoardObservationRpe() const
{
    for (const auto &[_, boardObs] : _boardObservations) {
        std::ignore = boardObs->calcRpeMean();
    }
}

void Calibration::Impl::calibrateBoardGroups()
{
    calcBoardPairPoses();
    _boardIdPairToTransform =
        calcAveragePosePairs(_boardPairPoses, _opts.useQuaternionAverage);
    initBoardGraph();
    initBoardGroups();
    initBoardGroupObservations();
    estimateBoardGroupPoses();
    calcBoardGroupObservationRpe();
    refineBoardGroupPoses();
    calcBoardGroupObservationRpe();
}

void Calibration::Impl::calcBoardPairPoses()
{
    _boardPairPoses.clear();

    for (const auto &[_0, camObs] : _camObservations) {
        if (camObs->_boardIds.size() <= 1ull) {
            continue;
        }

        const auto &boardObservations = camObs->_boardObservations;
        for (const auto &[_1, _boardObs1] : boardObservations) {
            auto boardObs1 = _boardObs1.lock();
            if (!boardObs1) {
                continue;
            }

            const auto &boardId1 = boardObs1->_boardId;
            for (const auto &[_2, _boardObs2] : boardObservations) {
                auto boardObs2 = _boardObs2.lock();
                if (!boardObs2) {
                    continue;
                }

                const auto &boardId2 = boardObs2->_boardId;
                if (boardId1 == boardId2) {
                    continue;
                }

                cv::Mat boardPairPose =
                    boardObs2->pose().inv() * boardObs1->pose();
                const auto boardPairId = std::make_pair(boardId1, boardId2);
                _boardPairPoses[boardPairId].push_back(boardPairPose);
            }
        }
    }
}

void Calibration::Impl::initBoardGraph()
{
    _covisBoardGraph = {};

    // Each board is a vertex if it has been observed at least once
    for (const auto &board : _boards) {
        if (board->hasObservation()) {
            _covisBoardGraph.addVertex(board->_id);
        }
    }

    for (const auto &[boardPairId, boardPairPoses] : _boardPairPoses) {
        _covisBoardGraph.addEdge(boardPairId.first, boardPairId.second,
                                 (1. / boardPairPoses.size()));
    }
}

// FIXME: Duplicated code in mergeObjects()
void Calibration::Impl::initBoardGroups()
{
    const std::vector<std::vector<int>> ccs =
        _covisBoardGraph.connectedComponents();

    LOG(INFO) << "Number of board group detected: " << ccs.size();

    for (size_t i{0}; i < ccs.size(); i++) {
        const auto &cc = ccs[i];
        LOG(INFO) << "Board group " << i << " has board count of " << cc.size();

        const auto &refBoardId = *std::ranges::min_element(cc);

        auto newBoardGroup = std::make_shared<BoardGroup>(refBoardId, i);

        // Compute the shortest path between the reference and the other board
        auto numPoints{0};
        for (const auto &boardId : cc) {
            newBoardGroup->insertBoard(_boards[boardId]);

            // Compute the transformation wrt. the reference board
            cv::Mat transform = cv::Mat_<double>::eye(4, 4);
            if (const auto transformPath =
                    _covisBoardGraph.shortestPathBetween(refBoardId, boardId);
                transformPath.size() >= 1ull) {
                for (size_t k{0}; k < transformPath.size() - 1; k++) {
                    cv::Mat current_trans =
                        _boardIdPairToTransform[std::make_pair(
                            transformPath[k], transformPath[k + 1])];
                    transform = transform * current_trans.inv();
                }
            }

            newBoardGroup->setBoardPose(transform, boardId);

            const auto transformedPoints =
                transformPoints(_boards[boardId]->points(),
                                newBoardGroup->boardOrientation(boardId),
                                newBoardGroup->boardPosition(boardId));
            // Make a indexing between board to object
            for (size_t k = 0; k < transformedPoints.size(); k++) {
                int cornerInd = k;
                const auto boardIdCornerInd =
                    std::make_pair(boardId, cornerInd);
                newBoardGroup->_boardIdCornerIdToObjId[boardIdCornerInd] =
                    numPoints;
                newBoardGroup->_boardIdCornerId.push_back(boardIdCornerInd);
                newBoardGroup->_points.push_back(transformedPoints[k]);
                numPoints++;
            }

            LOG(INFO) << "Add Board " << boardId << " to Board group " << i;
        }

        _boardGroups[i] = newBoardGroup;
    }
}

void Calibration::Impl::initBoardGroupObservation(int boardGroupId)
{
    auto &boardGroup = _boardGroups.at(boardGroupId);

    for (const auto &[camIdToFrameId, camObs] : _camObservations) {
        const auto &[camId, frameId] = camIdToFrameId;

        // Create BoardGroup observed in this camera observation.
        // Keep in mind that a single BoardGroup can be observed in one image
        auto boardGroupObs =
            std::make_shared<BoardGroupObs>(boardGroup, boardGroupId);

        // Check the boards observing this camera
        for (const auto &[_, _boardObs] : camObs->_boardObservations) {
            // Check if this board correspond to the object of interest
            auto boardObs = _boardObs.lock();
            if (boardObs && boardGroup->_boards.contains(boardObs->_boardId)) {
                boardGroupObs->insertBoardObservation(boardObs);
            }
        }

        if (boardGroupObs->_pointInds.empty()) {
            continue;
        }

        // Update the camobs//frame//camera//3DObject
        camObs->insertBoardGroupObservation(boardGroupObs);
        _frames[frameId]->insertBoardGroupObservation(boardGroupObs);
        _cameras[camId]->insertBoardGroupObservation(boardGroupObs);
        boardGroup->insertObservation(boardGroupObs);
        boardGroup->insertFrame(_frames[frameId]);
        insertBoardGroupObservation(boardGroupObs);
    }
}

void Calibration::Impl::initBoardGroupObservations()
{
    for (const auto &[boardGroupId, _] : _boardGroups) {
        initBoardGroupObservation(boardGroupId);
    }
}

void Calibration::Impl::saveDetectedKeypoints(const std::string &filename) const
{
    cv::FileStorage fs{filename, cv::FileStorage::WRITE};
    fs << "nb_camera" << static_cast<int>(_cameras.size());
    for (const auto &cam : _cameras) {
        const auto &camId = cam->_id;

        std::vector<int> frameIds;
        std::vector<std::string> imageFiles;
        std::vector<int> boardIds;
        std::vector<std::vector<cv::Point2f>> cornersList;
        std::vector<std::vector<int>> cornerIdsList;
        for (const auto &[_1, frame] : _frames) {
            const auto &frameId = frame->_id;
            const auto &imgFile = frame->_imagePaths[camId];

            for (size_t boardId{0}; boardId < _boards.size(); ++boardId) {
                for (const auto &[_2, boardObs] : _boardObservations) {
                    if (camId != boardObs->_cameraId ||
                        frameId != boardObs->_frameId ||
                        boardId != boardObs->_boardId) {
                        continue;
                    }

                    const auto &corners = boardObs->_corners;
                    const auto &cornerIds = boardObs->_cornerIds;

                    frameIds.push_back(frameId);
                    imageFiles.push_back(imgFile);
                    boardIds.push_back(boardId);
                    cornersList.push_back(corners);
                    cornerIdsList.push_back(cornerIds);
                }
            }
        }

        fs << std::format("camera_{}", camId);
        // clang-format off
        fs << "{"
           << "img_width" << cam->_imgSize.width
           << "img_height" << cam->_imgSize.height
           << "frame_idxs" << frameIds
           << "frame_paths" << imageFiles
           << "board_idxs" << boardIds
           << "pts_2d" << cornersList
           << "charuco_idxs" << cornerIdsList
           << "}";
        // clang-format on
    }
}

void Calibration::Impl::saveCameraParameters(const std::string &filename) const
{
    cv::FileStorage fs{filename, cv::FileStorage::WRITE};
    for (const auto &[camGroupId, camGroup] : _camGroups) {
        fs << "nb_camera" << static_cast<int>(_cameras.size());
        for (const auto &[_, _cam] : camGroup->_cameras) {
            auto cam = _cam.lock();
            if (!cam) {
                continue;
            }

            cv::Mat cameraMatrix, distortion;
            cam->getIntrinsics(cameraMatrix, distortion);

            fs << std::format("camera_{}", cam->_id);
            // clang-format off
            fs << "{"
               << "camera_matrix" << cameraMatrix
               << "distortion_vector" << distortion
               << "distortion_type" << cam->_type
               << "camera_group" << camGroupId
               << "img_width" << cam->_imgSize.width
               << "img_height" << cam->_imgSize.height
               << "camera_pose_matrix" << camGroup->cameraPose(cam->_id).inv()
               << "}";
            // clang-format on
        }
    }
}

void Calibration::Impl::saveBoardGroups(const std::string &filename) const
{
    cv::FileStorage fs{filename, cv::FileStorage::WRITE};
    for (const auto &[_0, boardGroup] : _boardGroups) {
        // x, y, z, Board Id, Corner Index
        std::vector<cv::Vec<float, 5>> points;
        points.reserve(boardGroup->_numPoints);
        for (const auto &[_1, _board] : boardGroup->_boards) {
            auto board = _board.lock();
            if (!board) {
                continue;
            }

            const auto &boardId = board->_id;
            for (size_t i{0}; i < board->numPoints(); i++) {
                std::pair<int, int> boardIdCornerInd =
                    std::make_pair(boardId, i);
                const auto &point =
                    boardGroup->_points
                        [boardGroup->_boardIdCornerIdToObjId[boardIdCornerInd]];
                points.emplace_back(point.x, point.y, point.z,
                                    static_cast<float>(boardId),
                                    static_cast<float>(i));
            }
        }

        fs << std::format("object_{}", boardGroup->_id);
        // clang-format off
        fs << "{"
           << "points" << cv::Mat{points}.reshape(1).t()
           << "}";
        // clang-format on
    }
}

void Calibration::Impl::saveBoardGroupPoses(const std::string &filename) const
{
    cv::FileStorage fs{filename, cv::FileStorage::WRITE};
    for (const auto &[_0, boardGroup] : _boardGroups) {
        std::vector<cv::Vec6d> poses;
        poses.reserve(boardGroup->_observations.size());
        for (const auto &[_1, _boardGroupObs] : boardGroup->_observations) {
            auto boardGroupObs = _boardGroupObs.lock();
            if (!boardGroupObs) {
                continue;
            }

            cv::Mat rvec, tvec;
            boardGroupObs->getPose(rvec, tvec);
            poses.emplace_back(rvec.at<double>(0), rvec.at<double>(1),
                               rvec.at<double>(2), tvec.at<double>(0),
                               tvec.at<double>(1), tvec.at<double>(2));
        }

        fs << std::format("object_{}", boardGroup->_id);
        // clang-format off
        fs << "{"
           << "poses" << cv::Mat{poses}.reshape(1).t()
           << "}";
        // clang-format on
    }
}

// FIXME: Duplicated code in computeAvgReprojectionError()
void Calibration::Impl::saveReprojectionErrors(
    const std::string &filename) const
{
    cv::FileStorage fs{filename, cv::FileStorage::WRITE};

    fs << "nb_camera_group" << static_cast<int>(_camGroups.size());

    cv::Mat frameIds;
    for (const auto &[_0, camGroup] : _camGroups) {
        const auto &camGroupId = camGroup->_id;

        fs << std::format("camera_group_{}", camGroupId);
        fs << "{";

        for (const auto &[_1, _frame] : camGroup->_frames) {
            auto frame = _frame.lock();
            if (!frame) {
                continue;
            }

            const auto &frameId = frame->_id;

            frameIds.push_back(frameId);

            fs << std::format("frame_{}", frameId);
            fs << "{";

            cv::Mat cameraIds;
            for (const auto &[_2, _camGroupObs] :
                 frame->_camGroupObservations) {
                auto camGroupObs = _camGroupObs.lock();
                if (!camGroupObs || camGroupId != camGroupObs->_camGroupId) {
                    continue;
                }

                for (const auto &[_3, _boardGroupObs] :
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
                    const auto &boardGroupId = boardGroupObs->_boardGroupId;
                    cameraIds.push_back(camId);

                    const auto &cornerIds = boardGroupObs->_pointInds;
                    const auto &imgPoints = boardGroupObs->_pixels;
                    std::vector<cv::Point3f> objPoints;
                    objPoints.reserve(cornerIds.size());
                    for (const auto &cornerId : cornerIds) {
                        objPoints.push_back(boardGroup->_points[cornerId]);
                    }

                    const auto transformedPoints = transformPoints(
                        objPoints, camGroupObs->objectOrientation(boardGroupId),
                        camGroupObs->objectPosition(boardGroupId));

                    std::vector<cv::Point2f> projectedPoints;
                    projectPointsWithDistortion(
                        transformedPoints, camGroup->cameraOrientation(camId),
                        camGroup->cameraPosition(camId), cam->cameraMatrix(),
                        cam->distortion(), projectedPoints, cam->_type);

                    cv::Mat errors = computeDistanceBetweenPoints(
                        imgPoints, projectedPoints);

                    fs << std::format("camera_{}", camId);
                    // clang-format off
                    fs << "{"
                       << "nb_pts" << static_cast<int>(cornerIds.size())
                       << "error_list" << errors
                       << "}";
                    // clang-format on
                }
            }
            fs << "camera_list" << cameraIds << "}";
        }
        fs << "frame_list" << frameIds << "}";
    }
}

void Calibration::Impl::saveDetection(const std::string &root) const
{
    const fs::path saveDir{root};
    fs::create_directories(saveDir);

    for (size_t camId{0}; camId < _cameras.size(); ++camId) {
        const auto camDir = saveDir / std::format("cam{}", camId);
        fs::create_directories(camDir);

        for (const auto &[frameId, frame] : _frames) {
            if (!frame->_imagePaths.contains(camId)) {
                continue;
            }

            auto image = cv::imread(frame->_imagePaths.at(camId));
            if (image.empty()) {
                continue;
            }

            for (const auto &[_1, _camGroupObs] :
                 frame->_camGroupObservations) {
                auto camGroupObs = _camGroupObs.lock();
                if (!camGroupObs) {
                    continue;
                }

                for (const auto &[_2, _boardGroupObs] :
                     camGroupObs->_boardGroupObservations) {
                    auto boardGroupObs = _boardGroupObs.lock();
                    if (!boardGroupObs || (boardGroupObs->_cameraId != camId)) {
                        continue;
                    }

                    auto boardGroup = boardGroupObs->_boardGroup.lock();
                    if (!boardGroup) {
                        continue;
                    }

                    for (const auto &point : boardGroupObs->_pixels) {
                        cv::drawMarker(image, point, CV_RGB(12, 233, 0),
                                       cv::MARKER_TILTED_CROSS, 3, 1,
                                       cv::LINE_AA);
                    }
                }
            }

            cv::imwrite((camDir / std::format("{:0>5}.png", frameId)).string(),
                        image);
        }
    }
}

void Calibration::Impl::saveReprojection(const std::string &root) const
{
    const auto detectedColor = CV_RGB(12, 233, 0);
    const auto projectedColor = CV_RGB(233, 11, 0);

    const fs::path saveDir{root};
    fs::create_directories(saveDir);

    for (size_t camId{0}; camId < _cameras.size(); ++camId) {
        const auto &cam = _cameras[camId];

        const auto camDir = saveDir / std::format("cam{}", camId);
        fs::create_directories(camDir);

        for (const auto &[frameId, frame] : _frames) {
            if (!frame->_imagePaths.contains(camId)) {
                continue;
            }

            auto image = cv::imread(frame->_imagePaths[camId]);
            if (image.empty()) {
                continue;
            }

            for (const auto &[_1, _camGroupObs] :
                 frame->_camGroupObservations) {
                auto camGroupObs = _camGroupObs.lock();
                if (!camGroupObs) {
                    continue;
                }

                for (const auto &[_2, _boardGroupObs] :
                     camGroupObs->_boardGroupObservations) {
                    auto boardGroupObs = _boardGroupObs.lock();
                    if (!boardGroupObs || boardGroupObs->_cameraId != camId) {
                        continue;
                    }

                    auto camGroup = camGroupObs->_camGroup.lock();
                    auto boardGroup = boardGroupObs->_boardGroup.lock();
                    if (!camGroup || !boardGroup) {
                        continue;
                    }

                    const cv::Mat camPose = camGroup->cameraPose(camId) *
                                            boardGroupObs->poseInGroup();
                    cv::Mat rvec, tvec;
                    Proj2RT(camPose, rvec, tvec);

                    const auto &detectedPoints = boardGroupObs->_pixels;
                    std::vector<cv::Point2f> projectedPoints;
                    {
                        const auto &cornerIds = boardGroupObs->_pointInds;
                        std::vector<cv::Point3f> objPoints;
                        objPoints.reserve(cornerIds.size());
                        for (const auto &cornerId : cornerIds) {
                            objPoints.push_back(boardGroup->_points[cornerId]);
                        }

                        projectPointsWithDistortion(
                            objPoints, rvec, tvec, cam->cameraMatrix(),
                            cam->distortion(), projectedPoints, cam->_type);
                    }

                    for (size_t i{0}; i < detectedPoints.size(); ++i) {
                        cv::drawMarker(image, detectedPoints[i], detectedColor,
                                       cv::MARKER_TILTED_CROSS, 3, 1,
                                       cv::LINE_AA);
                        cv::drawMarker(image, projectedPoints[i],
                                       projectedColor, cv::MARKER_TILTED_CROSS,
                                       3, 1, cv::LINE_AA);
                    }
                }
            }

            cv::imwrite((camDir / std::format("{:0>5}.png", frameId)).string(),
                        image);
        }
    }
}

void Calibration::Impl::insertBoardGroupObservation(
    std::shared_ptr<BoardGroupObs> boardGroupObs)
{
    _boardGroupObservations[_boardGroupObservations.size()] = boardGroupObs;
}

void Calibration::Impl::estimateBoardGroupPoses()
{
    for (const auto &[_, boardGroupObs] : _boardGroupObservations) {
        boardGroupObs->estimatePose(_opts.errorThreshold, _opts.numIterations);
    }
}

void Calibration::Impl::refineBoardGroupPoses()
{
    for (const auto &[_, boardGroup] : _boardGroups) {
        boardGroup->refineBoardGroup(_opts.numIterations);
    }
}

void Calibration::Impl::calcBoardGroupObservationRpe() const
{
    auto error{0.f};
    std::vector<float> errs;
    errs.reserve(_boardGroupObservations.size());
    for (const auto &[_, boardGroupObs] : _boardGroupObservations) {
        const auto err = boardGroupObs->calcRpeMean();
        errs.push_back(err);
        error += err;
    }
    error /= errs.size();

    LOG(INFO) << "Mean error of board group: " << error;
}

void Calibration::Impl::calibrateCameraGroups()
{
    calcCameraPairPoses();
    _cameraIdPairToTransform = calcAveragePosePairs(_cameraIdPairToTransforms,
                                                    _opts.useQuaternionAverage);
    initCameraGraph();
    initCameraGroup();
    initCameraGroupObservations();
    estimateBoardGroupInCameraGroupPoses();
    refineAllCameraGroupAndObjects();
}

void Calibration::Impl::calcCameraPairPoses()
{
    _cameraIdPairToTransforms.clear();

    for (const auto &[_0, frame] : _frames) {
        if (frame->_boardObservations.size() <= 1) {
            continue;
        }

        const auto &objObservations = frame->_boardGroupObservations;
        for (const auto &[_1, _objObs1] : objObservations) {
            auto objObs1 = _objObs1.lock();
            if (!objObs1) {
                continue;
            }

            const auto &camId1 = objObs1->_cameraId;
            const auto &objId1 = objObs1->_boardGroupId;
            const cv::Mat pose_cam_1 = objObs1->pose();
            for (const auto &[_2, _objObs2] : objObservations) {
                auto objObs2 = _objObs2.lock();
                if (!objObs2) {
                    continue;
                }

                const auto &camId2 = objObs2->_cameraId;
                const auto &objId2 = objObs2->_boardGroupId;

                // Two different cameras observe the same object
                if (camId1 != camId2 && objId1 == objId2) {
                    // not sure here ...
                    _cameraIdPairToTransforms[std::make_pair(camId1, camId2)]
                        .push_back(objObs2->pose() * pose_cam_1.inv());
                }
            }
        }
    }
}

void Calibration::Impl::initCameraGraph()
{
    _covisCameraGraph = {};

    // Each camera is a vertex if it has observed at least one object
    for (const auto &cam : _cameras) {
        if (!cam->_boardObservations.empty()) {
            _covisCameraGraph.addVertex(cam->_id);
        }
    }

    // Build the graph with cameras' pairs
    for (const auto &[camera_pair_idx, camera_poses_temp] :
         _cameraIdPairToTransforms) {
        _covisCameraGraph.addEdge(camera_pair_idx.first, camera_pair_idx.second,
                                  (1. / camera_poses_temp.size()));
    }
}

void Calibration::Impl::initCameraGroup()
{
    // Each connected components is a new camera group
    const std::vector<std::vector<int>> ccs =
        _covisCameraGraph.connectedComponents();

    LOG(INFO) << "Detected camera group count: " << ccs.size();

    for (size_t i{0}; i < ccs.size(); i++) {
        const auto &cc = ccs[i];

        LOG(INFO) << "Camera group " << i << " has camera count of "
                  << cc.size();

        const auto &refCamId = std::ranges::min(cc);

        auto newCameraGroup = std::make_shared<CameraGroup>(refCamId, i);
        for (const auto &camId : cc) {
            newCameraGroup->insertCamera(_cameras[camId]);

            // Compute the transformation between the reference cam and the
            // other cam in the group
            cv::Mat T = cv::Mat_<double>::eye(4, 4);
            if (const auto transformPath =
                    _covisCameraGraph.shortestPathBetween(refCamId, camId);
                transformPath.size() >= 1u) {
                for (size_t k{0}; k < transformPath.size() - 1; k++) {
                    cv::Mat T_k = _cameraIdPairToTransform[std::make_pair(
                        transformPath[k], transformPath[k + 1])];
                    // T = T * T_k.inv();
                    // T = T * T_k;
                    T = T_k * T;
                }
            }

            newCameraGroup->setCameraPose(T, camId);
        }

        _camGroups[i] = newCameraGroup;
    }
}

void Calibration::Impl::initCameraGroupObservation(int camGroupId)
{
    const auto &camIdsInGroup = _camGroups[camGroupId]->_camIds;

    for (const auto &[_0, frame] : _frames) {
        const auto &frameId = frame->_id;
        auto newCamGroupObs = std::make_shared<CameraGroupObs>(
            _camGroups[camGroupId], _opts.useQuaternionAverage);

        for (const auto &[_1, _objObs] : frame->_boardGroupObservations) {
            auto objObs = _objObs.lock();
            if (objObs) {
                if (std::ranges::find(camIdsInGroup, objObs->_cameraId) !=
                    camIdsInGroup.end()) {
                    // the camera is in the group so this object is visible
                    // in the cam group udpate the observation
                    newCamGroupObs->insertBoardGroupObservation(objObs);

                    // push the object observation in the camera group
                    _camGroups[camGroupId]->insertBoardGroupObservation(objObs);
                }
            }
        }

        if (!newCamGroupObs->_boardGroupObservations.empty()) {
            _camGroupObservations[std::make_pair(camGroupId, frameId)] =
                newCamGroupObs;
            _frames[frameId]->insertCameraGroupObservation(newCamGroupObs,
                                                           camGroupId);
            _camGroups[camGroupId]->insertFrame(_frames[frameId]);
        }
    }
}

void Calibration::Impl::initCameraGroupObservations()
{
    for (const auto &[_, camGroup] : _camGroups) {
        initCameraGroupObservation(camGroup->_id);
    }
}

void Calibration::Impl::estimateBoardGroupInCameraGroupPoses()
{
    for (const auto &[_, camGroup] : _camGroups) {
        camGroup->computeObjPoseInCameraGroup();
    }

    for (const auto &[_, camGroupObs] : _camGroupObservations) {
        camGroupObs->computeBoardGroupsPose();
    }
}

void Calibration::Impl::refineAllCameraGroupAndObjects()
{
    for (const auto &[_, camGroup] : _camGroups) {
        camGroup->refineCameraGroupAndBoardGroups(_opts.numIterations);
    }

    for (const auto &[_, obj] : _boardGroups) {
        obj->updateObjectPoints();
    }

    for (const auto &[_, camGroupObs] : _camGroupObservations) {
        camGroupObs->updateObjObsPose();
    }
}

void Calibration::Impl::refineAllCameraGroup()
{
    for (const auto &[_, camGroup] : _camGroups) {
        // camGroup->computeObjPoseInCameraGroup();
        camGroup->refineCameraGroup(_opts.numIterations);
    }

    for (const auto &[_, camGroupObs] : _camGroupObservations) {
        camGroupObs->updateObjObsPose();
    }
}

void Calibration::Impl::findPairObjectForNonOverlap()
{
    _camGroupIdPairToTransforms.clear();

    for (const auto &[camGroupId1, camGroup1] : _camGroups) {
        for (const auto &[camGroupId2, camGroup2] : _camGroups) {
            if (camGroupId1 == camGroupId2) {
                continue;
            }

            // Prepare the list of possible objects pairs
            std::map<std::pair<int, int>, unsigned int> count_pair_obs;
            for (const auto &[_0, obj1] : _boardGroups) {
                for (const auto &[_1, obj2] : _boardGroups) {
                    const auto &objId1 = obj1->_id;
                    const auto &objId2 = obj2->_id;
                    if (objId1 != objId2) {
                        count_pair_obs[std::make_pair(objId1, objId2)] = 0u;
                    }
                }
            }

            // Find frames in common
            std::map<int, std::shared_ptr<Frame>> commonFrames;
            {
                std::map<int, std::shared_ptr<Frame>> framesInGroup1;
                std::map<int, std::shared_ptr<Frame>> framesInGroup2;
                for (const auto &[frameId, _frame] : camGroup1->_frames) {
                    if (auto frame = _frame.lock()) {
                        framesInGroup1[frameId] = frame;
                    }
                }
                for (const auto &[frameId, _frame] : camGroup2->_frames) {
                    if (auto frame = _frame.lock()) {
                        framesInGroup2[frameId] = frame;
                    }
                }

                std::ranges::set_intersection(
                    framesInGroup1, framesInGroup2,
                    std::inserter(commonFrames, commonFrames.begin()));
            }

            // Iterate through the frames and count the occurrence of object
            // pair (to select the pair of object appearing the most)
            for (const auto &[_0, frame] : commonFrames) {
                // Find the index of the observation corresponding to the
                // groups in cam group obs

                // Access the objects 3D index for both groups
                auto common_frames_cam_group1_obs_ptr =
                    frame
                        ->_camGroupObservations[std::ranges::find(
                                                    frame->_camGroupIds,
                                                    camGroupId1) -
                                                frame->_camGroupIds.begin()]
                        .lock();
                auto common_frames_cam_group2_obs_ptr =
                    frame
                        ->_camGroupObservations[std::ranges::find(
                                                    frame->_camGroupIds,
                                                    camGroupId2) -
                                                frame->_camGroupIds.begin()]
                        .lock();
                if (!common_frames_cam_group1_obs_ptr ||
                    !common_frames_cam_group2_obs_ptr) {
                    continue;
                }

                for (const auto &[_1, _objObs1] :
                     common_frames_cam_group1_obs_ptr
                         ->_boardGroupObservations) {
                    auto objObs1 = _objObs1.lock();
                    if (!objObs1) {
                        continue;
                    }

                    const auto &objId1 = objObs1->_boardGroupId;
                    for (const auto &[_2, _objObs2] :
                         common_frames_cam_group2_obs_ptr
                             ->_boardGroupObservations) {
                        if (auto objObs2 = _objObs2.lock()) {
                            count_pair_obs[std::make_pair(
                                objId1, objObs2->_boardGroupId)]++;
                        }
                    }
                }
            }

            // find the pair of object with the maximum shared frames
            unsigned int maxCount = 0u;
            std::pair<int, int> maxPair = std::make_pair(0, 0);
            for (const auto &[pair, count] : count_pair_obs) {
                if (count > maxCount) {
                    maxPair = pair;
                    maxCount = count;
                }
            }

            // Save in the data structure
            LOG(INFO) << "Object " << maxPair.first << " and Object "
                      << maxPair.second << " share most frames of " << maxCount;
            _camGroupIdPairToTransforms[std::make_pair(camGroupId1,
                                                       camGroupId2)] = maxPair;
        }
    }
}

// Handeye calibration of a pair of non overlapping pair of group of cameras
void Calibration::Impl::initNonOverlapPair(int camGroupId1, int camGroupId2)
{
    std::shared_ptr<CameraGroup> camGroup1 = _camGroups[camGroupId1];
    std::shared_ptr<CameraGroup> camGroup2 = _camGroups[camGroupId2];

    const auto &[objectId1, objectId2] =
        _camGroupIdPairToTransforms[std::make_pair(camGroupId1, camGroupId2)];

    // Prepare the 3D objects
    std::shared_ptr<BoardGroup> object1 = _boardGroups[objectId1];
    std::shared_ptr<BoardGroup> object2 = _boardGroups[objectId2];

    // std::vector to store data for non-overlapping calibration
    // absolute pose stored to compute relative displacements
    std::vector<cv::Mat> pose_abs_1, pose_abs_2;
    cv::Mat repo_obj_1_2; // reprojected pts for clustering

    // Find frames in common
    std::map<int, std::shared_ptr<Frame>> commonFrames;
    {
        std::map<int, std::shared_ptr<Frame>> framesInGroup1;
        std::map<int, std::shared_ptr<Frame>> framesInGroup2;
        for (const auto &[frameId, _frame] : camGroup1->_frames) {
            if (auto frames = _frame.lock()) {
                framesInGroup1[frameId] = frames;
            }
        }
        for (const auto &[frameId, _frame] : camGroup2->_frames) {
            if (auto frames = _frame.lock()) {
                framesInGroup2[frameId] = frames;
            }
        }

        std::ranges::set_intersection(
            framesInGroup1, framesInGroup2,
            std::inserter(commonFrames, commonFrames.begin()));
    }

    for (const auto &[_, frame] : commonFrames) {
        // Find the index of the observation corresponding to the groups in
        // cam group obs
        int index_camgroup_1 =
            std::ranges::find(frame->_camGroupIds, camGroupId1) -
            frame->_camGroupIds.cbegin();
        int index_camgroup_2 =
            std::ranges::find(frame->_camGroupIds, camGroupId2) -
            frame->_camGroupIds.cbegin();

        // check if both objects of interest are in the frame
        auto camGroupObs1 =
            frame->_camGroupObservations[index_camgroup_1].lock();
        auto camGroupObs2 =
            frame->_camGroupObservations[index_camgroup_2].lock();
        if (!camGroupObs1 || !camGroupObs2) {
            continue;
        }

        const std::vector<int> &cam_group_obs_obj1 =
            camGroupObs1->_boardGroupIds;
        const std::vector<int> &cam_group_obs_obj2 =
            camGroupObs2->_boardGroupIds;
        auto it1 = std::ranges::find(cam_group_obs_obj1, objectId1);
        auto it2 = std::ranges::find(cam_group_obs_obj2, objectId2);

        if (it1 == cam_group_obs_obj1.end() ||
            it2 == cam_group_obs_obj2.end()) {
            continue;
        }

        // Reproject 3D objects in ref camera group
        auto cam_group_obs1_cam_group_ptr = camGroupObs1->_camGroup.lock();
        auto cam_group_obs2_cam_group_ptr = camGroupObs2->_camGroup.lock();
        if (!cam_group_obs1_cam_group_ptr || !cam_group_obs2_cam_group_ptr) {
            continue;
        }

        auto objObs1 =
            camGroupObs1
                ->_boardGroupObservations[it1 - cam_group_obs_obj1.begin()]
                .lock();
        auto objObs2 =
            camGroupObs2
                ->_boardGroupObservations[it2 - cam_group_obs_obj2.begin()]
                .lock();
        if (objObs1 && objObs2) {
            pose_abs_1.push_back(
                camGroupObs1->objectPose(objObs1->_boardGroupId));
            pose_abs_2.push_back(
                camGroupObs2->objectPose(objObs2->_boardGroupId));
        }
    }

    // Check if enough common poses are available:
    if (pose_abs_1.size() <= 3 || pose_abs_2.size() <= 3) {
        return;
    }

    cv::Mat pose_g1_g2;
    if (he_approach_ == 0) {
        constexpr int nb_cluster = 20;
        constexpr int nb_it_he = 200;
        pose_g1_g2 = handeyeBootstratpTranslationCalibration(
            nb_cluster, nb_it_he, pose_abs_1, pose_abs_2);
    }
    else {
        pose_g1_g2 = handeyeCalibration(pose_abs_1, pose_abs_2);
    }

    pose_g1_g2 = pose_g1_g2.inv();

    _camGroupIdPairToTransform[std::make_pair(camGroupId1, camGroupId2)] =
        pose_g1_g2;
    _camGroupIdPairToNumFrames[std::make_pair(camGroupId1, camGroupId2)] =
        pose_abs_1.size();
}

void Calibration::Impl::findPoseNoOverlapAllCamGroup()
{
    _camGroupIdPairToTransform.clear();

    for (const auto &[camGroupId1, _0] : _camGroups) {
        for (const auto &[camGroupId2, _1] : _camGroups) {
            if (camGroupId1 != camGroupId2) {
                initNonOverlapPair(camGroupId1, camGroupId2);
            }
        }
    }
}

void Calibration::Impl::initInterCamGroupGraph()
{
    _nonOverlapCamGroupGraph = {};

    for (const auto &[_, camGroup] : _camGroups) {
        if (!camGroup->_boardGroupObservations.empty()) {
            _nonOverlapCamGroupGraph.addVertex(camGroup->_id);
        }
    }

    // Create the graph
    for (const auto &[camgroup_pair_idx, _] : _camGroupIdPairToTransform) {
        _nonOverlapCamGroupGraph.addEdge(
            camgroup_pair_idx.first, camgroup_pair_idx.second,
            (1. / _camGroupIdPairToNumFrames.at(camgroup_pair_idx)));
    }
}

void Calibration::Impl::mergeCameraGroups()
{
    const std::vector<std::vector<int>> ccs =
        _nonOverlapCamGroupGraph.connectedComponents();

    std::map<int, std::shared_ptr<CameraGroup>> newCamGroups;
    for (size_t i{0}; i < ccs.size(); i++) {
        const auto &cc = ccs[i];
        const auto &refCamGroupId = *std::ranges::min_element(cc);
        const auto &refCamId = _camGroups[refCamGroupId]->_refCamId;

        // Recompute the camera pose in the referential of the reference
        // group pose of the cam group in the cam group
        std::map<int, cv::Mat> camGroupPosesToRef;
        for (const auto &camGroupId : cc) {
            // Compute the transformation wrt. the reference camera
            cv::Mat transform = cv::Mat_<double>::eye(4, 4);
            if (const auto transformPath =
                    _nonOverlapCamGroupGraph.shortestPathBetween(refCamGroupId,
                                                                 camGroupId);
                transformPath.size() >= 1ull) {
                for (size_t k{0}; k < transformPath.size() - 1; k++) {
                    const cv::Mat T_k =
                        _camGroupIdPairToTransform[std::make_pair(
                            transformPath[k], transformPath[k + 1])];
                    // transform = transform * current_trans.inv();
                    transform = transform * T_k;
                }
            }

            camGroupPosesToRef[camGroupId] = transform;
        }

        // Create the camera group
        auto newCameraGroup = std::make_shared<CameraGroup>(refCamId, i);
        for (const auto &[_0, camGroup] : _camGroups) {
            // Check if the current camera group belong to the final group
            if (std::ranges::find(cc, camGroup->_id) == cc.end()) {
                continue;
            }

            // Prepare the current group pose in the referential of the
            // final group
            const cv::Mat pose_in_final = camGroupPosesToRef[camGroup->_id];
            for (const auto &[_1, _cam] : camGroup->_cameras) {
                if (auto cam = _cam.lock()) {
                    // Update the pose in the referential of the final group
                    cv::Mat transform =
                        camGroup->cameraPose(cam->_id) * pose_in_final;
                    newCameraGroup->insertCamera(cam);
                    newCameraGroup->setCameraPose(transform, cam->_id);
                }
            }
        }

        newCamGroups[i] = newCameraGroup;
    }

    _camGroups.clear();
    _camGroups = newCamGroups;
}

void Calibration::Impl::mergeCameraGroupObservations()
{
    for (const auto &[_, frame] : _frames) {
        frame->_camGroupIds.clear();
        frame->_camGroupObservations.clear();
    }

    // Reinitialize all camera obserations
    _camGroupObservations.clear();
    for (const auto &[_, camGroup] : _camGroups) {
        initCameraGroupObservation(camGroup->_id);
    }
}

void Calibration::Impl::computeObjectsPairPose()
{
    _boardGroupIdPairToTransforms.clear();

    for (const auto &[_0, camGroupObs] : _camGroupObservations) {
        if (camGroupObs->_boardGroupIds.size() <= 1) {
            continue;
        }

        const auto &boardGroupObservations =
            camGroupObs->_boardGroupObservations;
        for (const auto &[_1, _boardGroupObs1] : boardGroupObservations) {
            auto boardGroupObs1 = _boardGroupObs1.lock();
            if (!boardGroupObs1) {
                continue;
            }

            const auto &boardGroupId1 = boardGroupObs1->_boardGroupId;
            const cv::Mat boardGroupPose1 =
                camGroupObs->objectPose(boardGroupId1);

            for (const auto &[_2, _boardGroupObs2] : boardGroupObservations) {
                auto boardGroupObs2 = _boardGroupObs2.lock();
                if (!boardGroupObs2) {
                    continue;
                }

                const auto &boardGroupId2 = boardGroupObs2->_boardGroupId;
                if (boardGroupId1 != boardGroupId2) {
                    _boardGroupIdPairToTransforms[std::make_pair(boardGroupId1,
                                                                 boardGroupId2)]
                        .push_back(
                            camGroupObs->objectPose(boardGroupId2).inv() *
                            boardGroupPose1);
                }
            }
        }
    }
}

void Calibration::Impl::initInterObjectsGraph()
{
    _covisBoardGroupGraph = {};

    // Each object is a vertex if it has been observed at least once
    for (const auto &[_, obj] : _boardGroups) {
        if (!obj->_observations.empty()) {
            _covisBoardGroupGraph.addVertex(obj->_id);
        }
    }

    for (const auto &[objectPairId, transforms] :
         _boardGroupIdPairToTransforms) {
        _covisBoardGroupGraph.addEdge(objectPairId.first, objectPairId.second,
                                      (1. / (transforms.size())));
    }
}

void Calibration::Impl::mergeBoardGroups()
{
    const std::vector<std::vector<int>> ccs =
        _covisBoardGroupGraph.connectedComponents();

    std::map<int, std::shared_ptr<BoardGroup>> newBoardGroups;
    for (size_t i{0}; i < ccs.size(); i++) {
        const auto &cc = ccs[i];
        const auto &refBoardGroupId = *std::ranges::min_element(cc);
        const auto &refBoardId = _boardGroups[refBoardGroupId]->_refBoardId;

        // recompute the board poses in the referential of the reference
        // object Used the graph to find the transformations of objects to
        // the reference object
        std::map<int, cv::Mat> boardGroupPosesToRef;
        auto numBoards{0};
        for (const auto &boardGroupId : cc) {
            numBoards += _boardGroups[boardGroupId]->_boards.size();

            // Compute the transformation wrt. the reference object
            cv::Mat T = cv::Mat_<double>::eye(4, 4);
            if (const auto transformPath =
                    _covisBoardGroupGraph.shortestPathBetween(refBoardGroupId,
                                                              boardGroupId);
                transformPath.size() >= 1ull) {
                for (size_t k = 0; k < transformPath.size() - 1; k++) {
                    const cv::Mat T_k =
                        _boardGroupIdPairsToTransform[std::make_pair(
                            transformPath[k], transformPath[k + 1])];
                    T = T * T_k.inv();
                }
            }

            boardGroupPosesToRef[boardGroupId] = T;
        }

        auto newBoardGroup = std::make_shared<BoardGroup>(refBoardId, i);
        auto numPoints{0};
        for (const auto &[_0, boardGroup] : _boardGroups) {
            const auto &boardGroupId = boardGroup->_id;
            // Check if the current object belong to the new object
            if (std::ranges::find(cc, boardGroupId) == cc.end()) {
                continue;
            }

            // Prepare the current object pose in the referential of the
            // merged object
            cv::Mat pose_in_merged = boardGroupPosesToRef[boardGroupId];
            // the object is in the merged group so we include its boards
            for (const auto &[_1, _board] : boardGroup->_boards) {
                auto board = _board.lock();
                if (!board) {
                    continue;
                }

                const auto &boardId = board->_id;
                // Update the pose to be in the referential of the
                // merged object
                cv::Mat pose_board_in_current_obj =
                    boardGroup->boardPose(boardId);
                cv::Mat transform = pose_in_merged * pose_board_in_current_obj;

                newBoardGroup->insertBoard(board);
                newBoardGroup->setBoardPose(transform, boardId);

                const auto transformedPoints = transformPoints(
                    board->points(), newBoardGroup->boardOrientation(boardId),
                    newBoardGroup->boardPosition(boardId));
                // Make a indexing between board to object
                for (size_t k = 0; k < transformedPoints.size(); k++) {
                    int cornerInd = k;
                    const auto boardIdCornerInd =
                        std::make_pair(boardId, cornerInd);
                    newBoardGroup->_boardIdCornerIdToObjId[boardIdCornerInd] =
                        numPoints;
                    newBoardGroup->_boardIdCornerId.push_back(boardIdCornerInd);
                    newBoardGroup->_points.push_back(transformedPoints[k]);
                    numPoints++;
                }
            }
        }

        newBoardGroups[i] = newBoardGroup;
    }

    _boardGroups.clear();
    _boardGroups = newBoardGroups;
}

void Calibration::Impl::mergeAllObjectObs()
{
    // First we erase all the object observation in the entire datastructure
    for (const auto &camera : _cameras) {
        camera->_visBoardGroupIds.clear();
        camera->_boardGroupObservations.clear();
    }

    for (const auto &[_, camGroup] : _camGroups) {
        camGroup->_visBoardGroupIds.clear();
        camGroup->_boardGroupObservations.clear();
    }

    for (const auto &[_, camGroupObs] : _camGroupObservations) {
        camGroupObs->_boardGroupIds.clear();
        camGroupObs->_boardGroupObservations.clear();
    }

    for (const auto &[_, camObs] : _camObservations) {
        camObs->_boardGroupObservations.clear();
        camObs->_boardGroupIds.clear();
    }

    for (const auto &[_, frame] : _frames) {
        frame->_boardGroupObservations.clear();
        frame->_boardGroupIds.clear();
    }

    for (const auto &[_, obj] : _boardGroups) {
        obj->_observations.clear();
    }

    _boardGroupObservations.clear();

    // Reinitialize all the 3D object
    for (const auto &it_object : _boardGroups) {
        (void)it_object;
        // Reinitialize all object obserations
        for (const auto &it : _boardGroups) {
            initBoardGroupObservation(it.first);
        }
    }
}

void Calibration::Impl::calcCameraGroupsRpe() const
{
    for (const auto &[_, camGroup] : _camGroups) {
        camGroup->calcRpeMean();
    }
}

void Calibration::Impl::merge3DObjects()
{
    initInterCamGroupGraph();
    estimateBoardGroupPoses();
    estimateBoardGroupInCameraGroupPoses();

    computeObjectsPairPose();
    _boardGroupIdPairsToTransform = calcAveragePosePairs(
        _boardGroupIdPairToTransforms, _opts.useQuaternionAverage);
    initInterObjectsGraph();
    calcCameraGroupsRpe();

    mergeBoardGroups();
    mergeAllObjectObs();
    mergeCameraGroupObservations();
    estimateBoardGroupPoses();
    estimateBoardGroupInCameraGroupPoses();
    refineAllCameraGroupAndObjects();
    calcCameraGroupsRpe();
}

double Calibration::Impl::calcOverallRpeMean() const
{
    cv::Mat frameIds;
    cv::Scalar total_avg_error_sum;
    int number_of_adds = 0;

    for (const auto &[_0, camGroup] : _camGroups) {
        const auto &camGroupId = camGroup->_id;

        for (const auto &[_1, _frame] : camGroup->_frames) {
            auto frame = _frame.lock();
            if (!frame) {
                continue;
            }

            frameIds.push_back(frame->_id);

            // TODO: Not used???
            cv::Mat cameraIds;
            for (const auto &[_2, _camGroupObs] :
                 frame->_camGroupObservations) {
                auto camGroupObs = _camGroupObs.lock();
                if (!camGroupObs || camGroupId != camGroupObs->_camGroupId) {
                    continue;
                }

                for (const auto &[_3, _boardGroupObs] :
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
                    const auto &boardGroupId = boardGroupObs->_boardGroupId;
                    cameraIds.push_back(camId);

                    const auto &cornerIds = boardGroupObs->_pointInds;
                    const auto &imgPoints = boardGroupObs->_pixels;
                    std::vector<cv::Point3f> objPoints;
                    objPoints.reserve(cornerIds.size());
                    for (const auto &cornerId : cornerIds) {
                        objPoints.push_back(boardGroup->_points[cornerId]);
                    }

                    const auto transformedPoints = transformPoints(
                        objPoints, camGroupObs->objectOrientation(boardGroupId),
                        camGroupObs->objectPosition(boardGroupId));

                    std::vector<cv::Point2f> projectedPoints;
                    projectPointsWithDistortion(
                        transformedPoints, camGroup->cameraOrientation(camId),
                        camGroup->cameraPosition(camId), cam->cameraMatrix(),
                        cam->distortion(), projectedPoints, cam->_type);

                    cv::Mat errors = computeDistanceBetweenPoints(
                        imgPoints, projectedPoints);
                    total_avg_error_sum += cv::mean(errors);
                    number_of_adds++;
                }
            }
        }
    }

    return total_avg_error_sum.val[0] / number_of_adds;
}

void Calibration::Impl::refineAllCameraGroupAndObjectsAndIntrinsic()
{
    for (const auto &[_, camGroup] : _camGroups) {
        camGroup->refineCameraGroupAndBoardGroupsAndIntrinsics(
            _opts.numIterations);
    }

    for (const auto &[_, boardGroup] : _boardGroups) {
        boardGroup->updateObjectPoints();
    }

    for (const auto &[_, camGroupObs] : _camGroupObservations) {
        camGroupObs->updateObjObsPose();
    }
}

bool Calibration::Options::isValid() const
{
    return !imageSize.empty() && !intrinsicModels.empty() &&
           !boardOptions.empty();
}

Calibration::Calibration(const Options &opts) : d(std::make_unique<Impl>())
{
    setOptions(opts);
}

Calibration::~Calibration() = default;

const Calibration::Options &Calibration::options() const { return d->_opts; }

void Calibration::setOptions(const Options &opts)
{
    if (!opts.isValid()) {
        return;
    }

    d->_opts = opts;

    // Initialize Cameras
    d->_cameras.clear();
    for (size_t i{0}; i < opts.intrinsicModels.size(); i++) {
        auto camera = std::make_shared<Camera>(i, opts.intrinsicModels[i]);
        camera->_imgSize = opts.imageSize;

        d->_cameras.push_back(camera);
    }

    // Initialize Boards
    d->_boards.clear();
    for (auto i{0}; i < opts.boardOptions.size(); ++i) {
        const auto &boardOpts = opts.boardOptions[i];

        Board::Options opts;
        opts.squareX = boardOpts.tagDim.width;
        opts.squareY = boardOpts.tagDim.height;
        opts.squareSize = boardOpts.tagSpacingRatio;
        opts.markerSize = boardOpts.tagSize;
        opts.dictionary = 11; // Don't care
        opts.startMarkerId = boardOpts.startTagId;

        d->_boards.push_back(std::make_shared<Board>(opts, i));
    }
}

bool Calibration::setFromJson(const std::string &json)
{
    try {
        const auto j = nlohmann::json::parse(json);

        Options opts;
        j.get_to(opts);
        setOptions(opts);
    }
    catch (const nlohmann::detail::parse_error &e) {
        LOG(WARNING) << "Failed to parse Multi-Camera-Multi-Board calibration "
                        "options json: "
                     << e.what();
        return false;
    }
    catch (const nlohmann::detail::out_of_range &e) {
        LOG(WARNING) << "Incompatible Multi-Camera-Multi-Board calibration "
                        "options json: "
                     << e.what();
        return false;
    }

    return true;
}

void Calibration::setupCameras(const std::vector<int> &intrinsicModels,
                               const cv::Size &imageSize)
{
}

void Calibration::setupBoards(
    const std::vector<AprilTag::Board::Options> &boardOptions)
{
    // ...
}

void Calibration::addBoardDetections(
    int frameId, int camId,
    const std::vector<AprilTag::Board::Detection> &detections, double timestamp,
    const std::string &filename)
{
    if (detections.size() != d->_boards.size() || camId >= d->_cameras.size()) {
        return;
    }

    for (size_t i{0}; i < detections.size(); ++i) {
        auto &detection = detections[i];
        if (detection.empty()) {
            continue;
        }

        if (detection.size() <
            static_cast<size_t>(std::round(d->_opts.minDetectRateInBoard *
                                           d->_boards[i]->numPoints()))) {
            continue;
        }

        const auto &corners = detection.corners;
        const auto &cornerIds = detection.cornerIds;

        // Check for colinnerarity. Meaning ???
        std::vector<cv::Point2f> pointsOnBoard;
        pointsOnBoard.reserve(cornerIds.size());
        for (const auto &cornerId : cornerIds) {
            const auto &point3 = d->_boards[i]->point(cornerId);
            pointsOnBoard.emplace_back(point3.x, point3.y);
        }

        auto _a{0.}, _b{0.}, _c{0.};
        const auto residual = calcLinePara(pointsOnBoard, _a, _b, _c);

        // Add the board if it passes the collinearity check
        if ((residual > d->_boards[i]->tagSize() * 0.1) && corners.size() > 4) {
            d->insertBoardObservation(camId, frameId, i, corners, cornerIds,
                                      timestamp, filename);
        }
    }
}

void Calibration::clearData()
{
    d->_frames.clear();

    d->_boardObservations.clear();
    d->_camObservations.clear();
    d->_boardGroups.clear();
    d->_boardGroupObservations.clear();
    d->_camGroups.clear();
    d->_camGroupObservations.clear();

    d->_boardPairPoses.clear();
    d->_boardIdPairToTransform.clear();
    d->_covisBoardGraph = {};

    d->_cameraIdPairToTransforms.clear();
    d->_cameraIdPairToTransform.clear();
    d->_covisCameraGraph = {};

    d->_boardGroupIdPairToTransforms.clear();
    d->_boardGroupIdPairsToTransform.clear();
    d->_covisBoardGroupGraph = {};

    d->_camGroupIdPairToTransforms.clear();
    d->_camGroupIdPairToTransform.clear();
    d->_camGroupIdPairToNumFrames.clear();
    d->_nonOverlapCamGroupGraph = {};
}

void Calibration::startCalibration()
{
    d->calibrateCameras();
    d->calibrateBoardGroups();
    d->calibrateCameraGroups();

    d->merge3DObjects();
    d->findPairObjectForNonOverlap();
    d->findPoseNoOverlapAllCamGroup();
    d->initInterCamGroupGraph();
    d->mergeCameraGroups();
    d->mergeCameraGroupObservations();
    d->merge3DObjects();
    d->initInterCamGroupGraph();
    d->mergeCameraGroups();
    d->mergeCameraGroupObservations();
    d->estimateBoardGroupPoses();
    d->estimateBoardGroupInCameraGroupPoses();
    d->refineAllCameraGroupAndObjects();
    if (!d->_opts.fixIntrinsics) {
        d->refineAllCameraGroupAndObjectsAndIntrinsic();
    }
    d->calcCameraGroupsRpe();
}

void Calibration::saveResults(const std::string &root) const
{
    const TaskDir taskDir{root};

    fs::create_directories(taskDir.resultDir());
    d->saveDetectedKeypoints(taskDir.keyPointsFile());
    d->saveCameraParameters(taskDir.calibratedCamerasFile());
    d->saveBoardGroups(taskDir.calibratedBoardsFile());
    d->saveBoardGroupPoses(taskDir.calibratedBoardPosesFile());
    d->saveReprojectionErrors(taskDir.reprojectionErrorFile());

    d->saveDetection(taskDir.detectionDir().string());
    d->saveReprojection(taskDir.reprojectionDir().string());
}

void Calibration::getExtrinsics(Eigen::Matrix4d &transform) const
{
    const auto camPose1 = d->_camGroups.at(0)->cameraPose(1);
    cv::cv2eigen(camPose1, transform);
}

std::pair<double, double> Calibration::samplePeriod() const
{
    // TODO: Do we really need to iterate all the frames? They are supposed in
    // chronological order.
    std::pair<double, double> period{math::kMaxDouble, math::kMinDouble};
    for (const auto &[_, frame] : d->_frames) {
        period.first = std::min(period.first, frame->_stamp);
        period.second = std::max(period.second, frame->_stamp);
    }
    return period;
}

bool Calibration::getCameraPoses(
    int camId, std::vector<double> &stamps,
    std::vector<Eigen::Vector3d> &translations,
    std::vector<Eigen::Quaterniond> &rotations) const
{
    if (camId >= d->_cameras.size() || camId < 0) {
        return false;
    }

    for (const auto &[_0, boardGroup] : d->_boardGroups) {
        for (const auto &[_1, _boardGroupObs] : boardGroup->_observations) {
            auto boardGroupObs = _boardGroupObs.lock();
            if (!boardGroupObs || boardGroupObs->_cameraId != camId) {
                continue;
            }

            // BoardGroup in Camera frame
            cv::Mat rvec, tvec;
            boardGroupObs->getPose(rvec, tvec);

            // Camera in BoardGroup frame
            cv::Mat rmat;
            cv::Rodrigues(rvec, rmat);
            const auto cam_rmat = rmat.t();
            const auto cam_tvec = -cam_rmat * tvec.t();

            const auto q = cv::Quatd::createFromRotMat(cam_rmat);
            rotations.emplace_back(q.w, q.x, q.y, q.z);

            Vector3d t;
            cv::cv2eigen(cam_tvec, t);
            translations.push_back(t);

            stamps.push_back(d->_frames.at(boardGroupObs->_frameId)->_stamp);
        }
    }

    return true;
}

const Camera *const Calibration::camera(int index) const
{
    if (index >= d->_cameras.size() || index < 0) {
        return nullptr;
    }

    return d->_cameras.at(index).get();
}

namespace key {
constexpr char kImageSize[]{"image_size"};
constexpr char kIntrinsicModels[]{"intrinsic_models"};
constexpr char kBoardOptions[]{"board_options"};
constexpr char kMinDetectRateInBoard[]{"min_detect_rate_in_board"};
constexpr char kErrorThreshold[]{"error_threshold"};
constexpr char kNumIterations[]{"num_iterations"};
constexpr char kFixIntrinsics[]{"fix_intrinsics"};
constexpr char kUseQuaternionAverage[]{"use_quaternion_average"};
} // namespace key

} // namespace tl::mcmb

namespace nlohmann {

using namespace tl::mcmb;

void to_json(json &j, const tmpfile_s::mcmb::Calibration::Options &opts)
{
    j[key::kImageSize] = opts.imageSize;
    j[key::kIntrinsicModels] = opts.intrinsicModels;
    j[key::kBoardOptions] = opts.boardOptions;
    j[key::kMinDetectRateInBoard] = opts.minDetectRateInBoard;
    j[key::kErrorThreshold] = opts.errorThreshold;
    j[key::kNumIterations] = opts.numIterations;
    j[key::kFixIntrinsics] = opts.fixIntrinsics;
    j[key::kUseQuaternionAverage] = opts.useQuaternionAverage;
}

void from_json(const json &j, tl::mcmb::Calibration::Options &opts)
{
    j.at(key::kImageSize).get_to(opts.imageSize);
    j.at(key::kIntrinsicModels).get_to(opts.intrinsicModels);
    j.at(key::kBoardOptions).get_to(opts.boardOptions);
    j.at(key::kMinDetectRateInBoard).get_to(opts.minDetectRateInBoard);
    j.at(key::kErrorThreshold).get_to(opts.errorThreshold);
    j.at(key::kNumIterations).get_to(opts.numIterations);
    j.at(key::kFixIntrinsics).get_to(opts.fixIntrinsics);
    j.at(key::kUseQuaternionAverage).get_to(opts.useQuaternionAverage);
}

} // namespace nlohmann
