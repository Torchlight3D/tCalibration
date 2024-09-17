#pragma once

#include "Decoder.h"
#include "EDInterface.h"
#include "Marker.h"
#include "QuadDetector.h"

namespace tl {
namespace stag {

class StagDetector
{
public:
    /**
     * Initializes StagDetector
     * @param libraryHD The library HD that is used. Possible values are
     * [11,&nbsp;13,&nbsp;15,&nbsp;17,&nbsp;19,&nbsp;21,&nbsp;23].
     * @param errorCorrection The amount of error correction that is going to be
     * used. Value needs to be in range
     * 0&nbsp;\<=&nbsp;errorCorrection&nbsp;\<=&nbsp;(HD-1)/2.
     */

    enum Family
    {
        HD11,
        HD13,
        HD15,
        HD17,
        HD19,
        HD21,
        HD23,
    };

    explicit StagDetector(int libraryHD, int errorCorrection = -1);

    void detectMarkers(const cv::Mat& inImage);

    const std::vector<Marker>& getMarkers() const;
    const std::vector<Quad>& getFalseCandidates() const;
    void logResults(const std::string& path = {}) const;

private:
    int errorCorrection;
    EDInterface edInterface;
    QuadDetector quadDetector;
    Decoder decoder;

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
};

} // namespace stag
} // namespace tl
