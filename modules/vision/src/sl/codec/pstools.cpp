#include "pstools.h"

#include <opencv2/core.hpp>

#include <tCore/Math>

namespace tl {

namespace pstools {

void calcPhaseVector(int length, int direction, float phase, float pitch,
                     cv::OutputArray _vec)
{
    using Pixel = cv::Vec3b;

    _vec.create(direction == Vertical ? length : 1, direction ? 1 : length,
                CV_8UC3);

    auto vec = _vec.getMat();
    vec.forEach<Pixel>([direction, phase, pitch](Pixel& px, const int pos[2]) {
        const auto& i = pos[direction];
        const auto amp =
            0.5f * (1.f + std::cos(2.f * pi_f * i / pitch - phase));
        px = Pixel(255 * amp, 255 * amp, 255 * amp);
    });
}

void phaseFromThreeFrames(cv::InputArrayOfArrays _frames, cv::OutputArray phase)
{
    CV_Assert(_frames.isMatVector());

    std::vector<cv::Mat> frames;
    _frames.getMatVector(frames);
    CV_Assert(frames.size() >= 3);

    const auto& I1 = frames[0];
    const auto& I2 = frames[1];
    const auto& I3 = frames[2];

    cv::phase(2.f * I1 - I3 - I2, sqrt3_f * (I2 - I3), phase);
}

void magnitudeFromThreeFrames(cv::InputArrayOfArrays _frames,
                              cv::OutputArray magnitude)
{
    CV_Assert(_frames.isMatVector());

    std::vector<cv::Mat> frames;
    _frames.getMatVector(frames);
    CV_Assert(frames.size() >= 3);

    const auto& I1 = frames[0];
    const auto& I2 = frames[1];
    const auto& I3 = frames[2];

    cv::Mat mag;
    cv::magnitude(2.f * I1 - I2 - I3, sqrt3_f * (I2 - I3), mag);

    // QUEST: Why???
    mag.convertTo(magnitude, CV_8U);
}

// Absolute phase and magnitude from N frames
std::vector<cv::Mat> getDFTComponents(const std::vector<cv::Mat>& frames)
{
    unsigned int N = frames.size();

    std::vector<cv::Mat> framesReverse = frames;
    std::reverse(framesReverse.begin(), framesReverse.end());

    // DFT approach
    cv::Mat I;
    cv::merge(frames, I);
    unsigned int w = I.cols;
    unsigned int h = I.rows;
    I = I.reshape(1, h * w);
    I.convertTo(I, CV_32F);
    cv::Mat fI;
    cv::dft(I, fI, cv::DFT_ROWS + cv::DFT_COMPLEX_OUTPUT);
    fI = fI.reshape(N * 2, h);

    std::vector<cv::Mat> fIcomp;
    cv::split(fI, fIcomp);

    return fIcomp;
}

// Phase unwrapping by means of a phase cue
cv::Mat unwrapWithCue(const cv::Mat up, const cv::Mat upCue,
                      unsigned int nPhases)
{
    // Determine number of jumps
    cv::Mat P = (upCue * nPhases - up) / (2 * pi_f);

    // Round to integers
    P.convertTo(P, CV_8U);
    P.convertTo(P, CV_32F);

    // Add to phase
    cv::Mat upUnwrapped = up + P * 2 * pi_f;

    // Scale to range [0; 2pi]
    upUnwrapped *= 1.0 / nPhases;

    return upUnwrapped;
}

} // namespace pstools

} // namespace tl
