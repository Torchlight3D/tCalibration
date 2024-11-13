#pragma once

#include <map>

#include <opencv2/core/types.hpp>

namespace tl::mcmb {

class BoardObs;
class Frame;

// Geometry and layout of a Calibration board.
// Soft linked to:
// + The Frames observe this Board
// + The Board Observations of this Boards
//
// FIXME:
// 1. This class was created for CharucoBoard at first, but that's a bad design.
// It should support any types of the board shaped targets.
class Board
{
public:
    struct Options
    {
        float squareSize;
        float markerSize;
        int squareX, squareY;
        int startMarkerId;
        int dictionary;
    };

    Board() = delete;
    Board(const Options& options, int boardId);

    size_t numPoints() const;
    float tagSize() const;

    const std::vector<cv::Point3f>& points() const;
    const cv::Point3f& point(int pointId) const;

    void insertFrame(std::shared_ptr<Frame> frame);
    void insertObservation(std::shared_ptr<BoardObs> observation);
    size_t numObservations() const;
    bool hasObservation() const;
    void clear();

public:
    int _id;

private:
    Options _opts;

    std::vector<cv::Point3f> _points;
    // Local indexing from 0 to N-1.
    std::vector<int> _pointIds;

    std::map<int, std::weak_ptr<Frame>> _frames;
    std::map<int, std::weak_ptr<BoardObs>> _boardObservations;
};

} // namespace tl::mcmb
