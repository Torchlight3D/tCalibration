#pragma once

#include <string>

#include "ED/EDLines.h"
#include "ED/EdgeMap.h"
#include "QuadDetector.h"
#include "Marker.h"

class Drawer
{
    void colorAPixel(cv::Mat& img, int x, int y, cv::Scalar color,
                     int dotWidth);

public:
    // draws edge segments
    void drawEdgeMap(const std::string& path, cv::Mat image, EdgeMap* edgeMap);

    // draws line segments
    void drawLines(const std::string& path, cv::Mat image, EDLines* edLines);

    // draws corners (intersections of line segments)
    void drawCorners(const std::string& path, cv::Mat image,
                     const std::vector<std::vector<Corner>>& cornerGroups);

    // draws quads
    void drawQuads(const std::string& path, cv::Mat image,
                   const std::vector<Quad>& quads);

    // draws markers
    void drawMarkers(const std::string& path, cv::Mat image,
                     const std::vector<Marker>& markers);

    // draws refined markers and their ellipses
    void drawEllipses(const std::string& path, cv::Mat image,
                      const std::vector<Marker>& markers);
};
