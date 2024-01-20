#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>

namespace tl {

class MultiCheckerBoardDetector
{
public:
    struct Options
    {
        float scoreThreshold = 0.01f;
        float lamda = 0.5f;
        bool doSubPix = true;

        Options() {}
    };

    explicit MultiCheckerBoardDetector(const Options &options = {});
    ~MultiCheckerBoardDetector();

    void setup(const Options &options);

    struct Corners
    {
        std::vector<cv::Point2f> pt;
        std::vector<cv::Vec2f> v1;
        std::vector<cv::Vec2f> v2;
        std::vector<float> score;
    };

    void findCorners(const cv::Mat &image, Corners &corners);

    void chessboardsFromCorners(const Corners &corners,
                                std::vector<cv::Mat> &chessboards);

    void drawChessboards(cv::Mat &image,
                         const std::vector<cv::Mat> &chessboards,
                         const Corners &corners) const;

private:
    class Impl;
    const std::unique_ptr<Impl> d;

    friend Impl;
};

} // namespace tl
