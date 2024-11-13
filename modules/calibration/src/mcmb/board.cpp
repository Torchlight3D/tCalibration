#include "board.h"

#include <glog/logging.h>

#include <tVision/Target/AprilTag>

#include "frame.h"

namespace tl::mcmb {

Board::Board(const Options &opts, int boardId) : _opts(opts), _id(boardId)
{
    AprilTag::Board::Options boardOptions;
    boardOptions.tagDim = {opts.squareX, opts.squareY};
    boardOptions.tagSize = opts.markerSize;
    boardOptions.tagSpacingRatio = opts.squareSize;
    boardOptions.startTagId = opts.startMarkerId;
    boardOptions.useLocalCornerIndex = true;

    AprilTag::Board board{boardOptions};
    _points.swap(board.points);
    _pointIds.swap(board.ids);
}

size_t Board::numPoints() const { return _points.size(); }

float Board::tagSize() const { return _opts.squareSize; }

const std::vector<cv::Point3f> &Board::points() const { return _points; }

const cv::Point3f &Board::point(int pointId) const
{
    // Local Id
    return _points[pointId];

    // Global Id
    // return _points[pointId - _pointIds[0]];
}

void Board::insertFrame(std::shared_ptr<Frame> frame)
{
    if (const auto &frameId = frame->_id; !_frames.contains(frameId)) {
        _frames.insert({frameId, frame});
    }
}

void Board::insertObservation(std::shared_ptr<BoardObs> boardObs)
{
    _boardObservations[_boardObservations.size()] = boardObs;
}

size_t Board::numObservations() const { return _boardObservations.size(); }

bool Board::hasObservation() const { return !_boardObservations.empty(); }

void Board::clear()
{
    _frames.clear();
    _boardObservations.clear();
}
} // namespace tl::mcmb
