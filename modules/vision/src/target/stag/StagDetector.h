#pragma once

#include "EDInterface.h"
#include "QuadDetector.h"
#include "Drawer.h"
#include "Marker.h"
#include "Decoder.h"
#include "PoseRefiner.h"

class StagDetector
{
    int errorCorrection;
    EDInterface edInterface;
    QuadDetector quadDetector;
    Drawer drawer;
    Decoder decoder;
    PoseRefiner poseRefiner;

    std::vector<cv::Mat> codeLocs;
    std::vector<cv::Mat> blackLocs;
    std::vector<cv::Mat> whiteLocs;

    cv::Mat image;
    std::vector<Marker> markers;
    std::vector<Quad> falseCandidates;

    // take readings from 48 code locations, 12 black border locations, and 12
    // white border locations thresholds and converts to binary code
    Codeword readCode(const Quad& q);
    void fillCodeLocations();
    cv::Mat createMatFromPolarCoords(double radius, double radians,
                                     double circleRadius);

public:
    /**
     * Initializes StagDetector
     * @param libraryHD The library HD that is used. Possible values are
     * [11,&nbsp;13,&nbsp;15,&nbsp;17,&nbsp;19,&nbsp;21,&nbsp;23].
     * @param errorCorrection The amount of error correction that is going to be
     * used. Value needs to be in range
     * 0&nbsp;\<=&nbsp;errorCorrection&nbsp;\<=&nbsp;(HD-1)/2.
     */
    explicit StagDetector(int libraryHD, int errorCorrection = -1);

    const std::vector<Marker>& getMarkers();

    const std::vector<Quad>& getFalseCandidates();

    /**
     * Detects markers in given image.
     * @param inImage OpenCV Matrix of input image.
     */
    void detectMarkers(const cv::Mat& inImage);

    /**
     * Logs results to path (creates images for every detection step).
     * @param path Path where the images are going to be created. Directory must
     * exist, otherwise images cannot be saved.
     */
    void logResults(const std::string& path = "");
};
