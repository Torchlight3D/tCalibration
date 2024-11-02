#pragma once

#include <opencv2/core/mat.hpp>

namespace AprilTags {

class UnionFindSimple;

// Represents an edge between adjacent pixels in the image.
// The edge is encoded by the indices of the two pixels. Edge cost is
// proportional to the difference in local orientations.
class Edge
{
public:
    // Minimum intensity gradient for an edge to be recognized
    inline static constexpr float minMag = 0.004f;

    // 30 degrees = maximum acceptable difference in local orientations
    inline static constexpr float maxEdgeCost = 30.f * float(M_PI) / 180.f;

    // was 10000
    inline static constexpr int WEIGHT_SCALE = 100;

    // Theta threshold for merging edges
    inline static constexpr float thetaThresh = 100.f;

    // Magnitude threshold for merging edges
    inline static constexpr float magThresh = 1200.f;

    int pixelIdxA;
    int pixelIdxB;
    int cost;

    Edge() : pixelIdxA(), pixelIdxB(), cost() {}
    Edge(int pixelIdxA, int pixelIdxB, int cost)
        : pixelIdxA(pixelIdxA), pixelIdxB(pixelIdxB), cost(cost)
    {
    }

    inline bool operator<(const Edge &other) const
    {
        return (cost < other.cost);
    }

    // Cost of an edge between two adjacent pixels; -1 if no edge here An edge
    // exists between adjacent pixels if the magnitude of the intensity gradient
    // at both pixels is above threshold.  The edge cost is proportional to the
    // difference in the local orientation at the two pixels.  Lower cost is
    // better.  A cost of -1 means there is no edge here (intensity gradien fell
    // below threshold).
    static int edgeCost(float theta0, float theta1, float mag1);

    // Calculates and inserts up to four edges into 'edges', a vector of Edges.
    static void calcEdges(float theta0, int x, int y, const cv::Mat &theta,
                          const cv::Mat &mag, std::vector<Edge> &edges);

    // Process edges in order of increasing cost, merging clusters if we can do
    // so without exceeding the thetaThresh.
    static void mergeEdges(const std::vector<Edge> &edges, UnionFindSimple &uf,
                           float tmin[], float tmax[], float mmin[],
                           float mmax[]);
};

} // namespace AprilTags
