#pragma once

#include "codec.h"

namespace tl {

// NOTE: This is the same as CodecPhaseShift2x3
class EncoderCalibration : public Encoder
{
public:
    EncoderCalibration(unsigned int _screenCols, unsigned int _screenRows,
                       CodecDir _dir);
    // Encoding
    cv::Mat getEncodingPattern(unsigned int depth) const override;

private:
    std::vector<cv::Mat> patterns;
};

class DecoderCalibration : public Decoder
{
public:
    DecoderCalibration(unsigned int _screenCols, unsigned int _screenRows,
                       CodecDir _dir);

    // Decoding
    void setFrame(unsigned int depth, cv::Mat frame) override;
    void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                      cv::Mat &shading) const override;

private:
    std::vector<cv::Mat> _frames;
};

} // namespace tl
