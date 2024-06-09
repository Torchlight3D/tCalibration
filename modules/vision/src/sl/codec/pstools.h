#pragma once

#include <opencv2/core/mat.hpp>

namespace tl {

namespace pstools {

enum Direction
{
    Vertical,
    Horizontal,

};

void calcPhaseVector(int length, int direction, float phase, float pitch,
                     cv::OutputArray vec);

// TODO: Not implemented
cv::Mat computePhaseVectorDithered(unsigned int length, float phase,
                                   float pitch);
void calcPhaseVectorDithered(int length, int direction, float phase,
                             float pitch, cv::OutputArray vec);

void phaseFromThreeFrames(cv::InputArrayOfArrays frames, cv::OutputArray phase);

void magnitudeFromThreeFrames(cv::InputArrayOfArrays frames,
                              cv::OutputArray magnitude);

std::vector<cv::Mat> getDFTComponents(const std::vector<cv::Mat>& frames);

cv::Mat unwrapWithCue(const cv::Mat up, const cv::Mat upCue,
                      unsigned int nPhases);
} // namespace pstools

} // namespace tl
