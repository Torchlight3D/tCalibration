#pragma once

#include "digitalmarkermodel.hpp"
#include "ellipsepoint.hpp"
#include "markerdetected.hpp"
#include "slotfitter.hpp"

namespace tl {
namespace runetag {

struct DetectorStats
{
    // Number of total pairs checked
    unsigned int num_pairs_checked = 0;

    // Number of total pairs for which two big ellipses can be fitted
    unsigned int num_pairs_fitted = 0;

    // Number of total pairs for which two big ellipses cannot be fitted
    unsigned int num_pairs_not_fitted = 0;

    // Number of total pairs for which two big ellipses cannot be refitted (too
    // few points to re-fit a circle in least-square sense)
    unsigned int num_pairs_not_refitted = 0;

    // Number of candidate marker which has been tested against a known model
    unsigned int num_models_tested = 0;

    DetectorStats() {}
};

class MarkerDetector
{
public:
    MarkerDetector(const cv::Mat& _intrinsics);
    virtual ~MarkerDetector();

    int addModelsFromFile(std::string filename);

    int detectMarkers(const std::vector<cv::RotatedRect>& ellipses,
                      std::vector<MarkerDetected>& markers_detected);
    int detectMarkers(const std::vector<cv::RotatedRect>& ellipses,
                      std::vector<MarkerDetected>& markers_detected,
                      DetectorStats& stats);

    class MarkerDetectorException : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

public:
    cv::Mat dbgimage;

private:
    void toEllipsePoints(const std::vector<cv::RotatedRect>& ellipses);
    bool findModel(const std::vector<MarkerDetected>& possible_markers,
                   std::vector<MarkerDetected>& markers_detected);
    bool tryFit(SlotFitter& sf, std::map<int, MarkerDetected>& markers_by_id);

private:
    cv::Mat intrinsics;
    std::map<long, DigitalMarkerModel> models;
    std::vector<EllipsePoint> ellipsePoints;
    unsigned int min_pts_for_level;
};

} // namespace runetag
} // namespace tl
