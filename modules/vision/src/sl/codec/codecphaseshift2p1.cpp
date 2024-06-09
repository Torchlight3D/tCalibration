#include "codecphaseshift2p1.h"

#include <iostream>

#include <opencv2/imgproc.hpp>

#include <tCore/Math>

#include "pstools.h"

namespace tl {

// Encoder
EncoderPhaseShift2p1::EncoderPhaseShift2p1(unsigned int _screenCols,
                                           unsigned int _screenRows,
                                           CodecDir _dir)
    : Encoder(_screenCols, _screenRows, _dir)
{
    N = 3;
    patterns.reserve(N);

    // Precompute encoded patterns
    constexpr auto kDirection = pstools::Horizontal;
    for (const auto &deg : {0.f, 90.f}) {
        cv::Mat pattern;
        pstools::calcPhaseVector(screenCols, kDirection, pi_f * (deg / 180.f),
                                 screenCols, pattern);
        patterns.push_back(pattern);
    }

    // Flat image
    patterns.push_back(cv::Mat(5, 5, CV_8UC3, cv::Scalar::all(127)));
}

cv::Mat EncoderPhaseShift2p1::getEncodingPattern(unsigned int depth) const
{
    return patterns[depth];
}

// Decoder
DecoderPhaseShift2p1::DecoderPhaseShift2p1(unsigned int _screenCols,
                                           unsigned int _screenRows,
                                           CodecDir _dir)
    : Decoder(_screenCols, _screenRows)
{
    N = 3;
    frames.resize(N);

    lastShading = new cv::Mat_<float>();
}

DecoderPhaseShift2p1::~DecoderPhaseShift2p1()
{
    cv::FileStorage fs("shiftHistory.xml", cv::FileStorage::WRITE);
    if (!fs.isOpened())
        std::cerr << "Could not write shiftHistory.xml" << std::endl;

    fs << "shiftHistory" << cv::Mat(shiftHistory);
    fs.release();
}

void DecoderPhaseShift2p1::setFrame(unsigned int depth, cv::Mat frame)
{
    frames[depth] = frame;
}

void DecoderPhaseShift2p1::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                                        cv::Mat &shading) const
{
    cv::Mat_<float> I1(frames[0]);
    cv::Mat_<float> I2(frames[1]);
    cv::Mat_<float> I3(frames[2]);

    if (lastShading->empty())
        *lastShading = I3;

    cv::Point2f shift;

    shift = cv::phaseCorrelate(*lastShading, I3);

    *lastShading = I3;

    // Shift input images according to global shift
    cv::Point2f center3(I3.cols / 2.0, I3.rows / 2.0);
    cv::Point2f center1(center3.x + 0.333 * shift.x,
                        center3.y + 0.333 * shift.y);
    cv::Point2f center2(center3.x - 0.333 * shift.x,
                        center3.y - 0.333 * shift.y);

    // Cannot process in-place when shift is positive
    cv::Mat_<float> I1Copy = I1.clone();
    cv::Mat_<float> I2Copy = I2.clone();

    cv::getRectSubPix(I1Copy, I1.size(), center1, I1);
    cv::getRectSubPix(I2Copy, I2.size(), center2, I2);

    cv::phase(I1 - I3, I2 - I3, up);
    up *= screenCols / (2 * pi_f);

    cv::GaussianBlur(up, up, cv::Size(0, 0), 3, 3);

    cv::Mat mag;
    cv::magnitude(I1 - I3, I2 - I3, mag);

    shading = 2.0 * frames[2];

    // Create mask from modulation image and erode
    mask.create(shading.size(), cv::DataType<bool>::type);
    mask.setTo(true);
    mask = (shading > 80) & (shading < 254);

    //    cv::Mat flow;
    //    cv::calcOpticalFlowSF(I1, I2, flow, 1, 3, 1);
    //    cv::Ptr<cv::DenseOpticalFlow> tvl1flow = cv::createOptFlow_DualTVL1();
    //    tvl1flow->calc(I1, I2, flow);

    std::cout << shift << std::endl;

    // draw vector on shading
    cv::Size frameSize = lastShading->size();
    cv::Point2f center(frameSize.width / 2, frameSize.height / 2);
    // cv::line(shading, center, center+30*shift, cv::Scalar(255), 5);

    // mask outlier-prone regions
    cv::Mat dx, dy;
    cv::Sobel(I3, dx, -1, 1, 0, 3);
    cv::Sobel(I3, dy, -1, 0, 1, 3);
    cv::Mat edgesShading = abs(dx) + abs(dy);

    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);
    cv::Mat edgesUp = abs(dx) + abs(dy);

    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);

    mask = mask & (edgesShading < 100) & (edgesUp < 130);
}

} // namespace tl
