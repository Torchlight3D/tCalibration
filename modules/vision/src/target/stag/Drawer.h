#pragma once

#include "ED/EDLines.h"
#include "ED/EdgeMap.h"
#include "QuadDetector.h"
#include "Marker.h"

namespace tl {
namespace stag {

void drawEdgeMap(cv::InputOutputArray image, EdgeMap* edgeMap);

void drawLines(cv::InputOutputArray image, EDLines* edLines);

void drawCorners(cv::InputOutputArray image,
                 const std::vector<std::vector<Corner>>& cornerGroups);

void drawQuads(cv::InputOutputArray image, const std::vector<Quad>& quads);

void drawMarkers(cv::InputOutputArray image,
                 const std::vector<Marker>& markers);

void drawEllipses(cv::InputOutputArray image,
                  const std::vector<Marker>& markers);

}; // namespace stag
} // namespace tl
