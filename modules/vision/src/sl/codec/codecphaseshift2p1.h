#pragma once

#include "codec.h"

namespace tl {

class EncoderPhaseShift2p1 : public Encoder
{
public:
    EncoderPhaseShift2p1(unsigned int _screenCols, unsigned int _screenRows,
                         CodecDir _dir);

    // Encoding
    cv::Mat getEncodingPattern(unsigned int depth) const override;

private:
    std::vector<cv::Mat> patterns;
};

class DecoderPhaseShift2p1 : public Decoder
{
public:
    DecoderPhaseShift2p1(unsigned int _screenCols, unsigned int _screenRows,
                         CodecDir _dir);
    ~DecoderPhaseShift2p1();

    // Decoding
    void setFrame(unsigned int depth, cv::Mat frame) override;
    void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                      cv::Mat &shading) const override;

private:
    std::vector<cv::Mat> frames;
    std::vector<cv::Point2d> shiftHistory;
    cv::Mat_<float> *lastShading;
};

} // namespace tl
