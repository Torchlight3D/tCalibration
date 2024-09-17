#pragma once

#include <opencv2/core/mat.hpp>

#include "ED/EDLines.h"
#include "ED/EdgeMap.h"

class EDInterface
{
    EdgeMap* edgeMap = NULL;
    EDLines* edLines = NULL;

public:
    ~EDInterface()
    {
        delete edgeMap;
        delete edLines;
    }

    // runs EDPF and EDLines, keeps the results in memory
    void runEDPFandEDLines(const cv::Mat& image);

    EdgeMap* getEdgeMap() const;
    EDLines* getEDLines() const;

    // ensures that when going from the start to end of a line segment,
    // right-hand side is darker
    void correctLineDirection(const cv::Mat& image, LineSegment& ls);

    // calculates the intersection of two line segments
    cv::Point2d intersectionOfLineSegments(const LineSegment& line1,
                                           const LineSegment& line2);
};
