#include "codecphaseshift3unwrap.h"

#include <opencv2/imgproc.hpp>

#include <tCore/Math>

#include "phaseunwrap.h"
#include "pstools.h"

namespace tl {

namespace {
constexpr unsigned int nPhases = 4;
}

// Encoder

EncoderPhaseShift3Unwrap::EncoderPhaseShift3Unwrap(unsigned int _screenCols,
                                                   unsigned int _screenRows,
                                                   CodecDir _dir)
    : Encoder(_screenCols, _screenRows, _dir)
{
    // Set N
    this->N = 3;

    // Precompute encoded patterns

    // Horizontally encoding patterns
    for (unsigned int i = 0; i < 3; i++) {
        float phase = 2.0 * pi_f / 3.0 * i;
        float pitch = (float)screenCols / (float)nPhases;
        cv::Mat pattern;
        pstools::calcPhaseVector(screenCols, 1, phase, pitch, pattern);
        patterns.push_back(pattern);
    }
}

cv::Mat EncoderPhaseShift3Unwrap::getEncodingPattern(unsigned int depth) const
{
    return patterns[depth];
}

// Decoder
DecoderPhaseShift3Unwrap::DecoderPhaseShift3Unwrap(unsigned int _screenCols,
                                                   unsigned int _screenRows,
                                                   CodecDir _dir)
    : Decoder(_screenCols, _screenRows)
{
    this->N = 3;
    frames.resize(N);
}

void DecoderPhaseShift3Unwrap::setFrame(unsigned int depth, cv::Mat frame)
{
    frames[depth] = frame;
}

void DecoderPhaseShift3Unwrap::decodeFrames(cv::Mat &up, cv::Mat &vp,
                                            cv::Mat &mask,
                                            cv::Mat &shading) const
{
    // Calculate multiple phase image
    pstools::phaseFromThreeFrames(frames, up);

    // Calculate modulation
    pstools::magnitudeFromThreeFrames(frames, shading);

    // Create mask from modulation image
    mask = shading > 25;

    // Unwrap multiple phase image
    cv::Mat quality = phaseunwrap::createqualitymap(up, mask);

    // Blurred quality map
    cv::GaussianBlur(quality, quality, cv::Size(0, 0), 3, 3);

    std::vector<float> thresholds =
        phaseunwrap::computethresholds(quality, mask);

    phaseunwrap::unwrap(up, quality, mask, thresholds);

    up += 3.0 * 2.0 * pi_f;
    up *= screenCols / (2.0 * pi_f * nPhases);
}

} // namespace tl
