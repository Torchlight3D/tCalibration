
#include <opencv2/core/mat.hpp>

namespace phasecorrelation {

// FIXME: Many of these functions are copies from OpenCV, use the orignal ones.
void magSpectrums(cv::InputArray _src, cv::OutputArray _dst);
void divSpectrums(cv::InputArray _srcA, cv::InputArray _srcB,
                  cv::OutputArray _dst, int flags, bool conjB);
void fftShift(cv::InputOutputArray _out);
cv::Point2d weightedCentroid(cv::InputArray _src, cv::Point peakLocation,
                             cv::Size weightBoxSize, double* response);
cv::Point2d phaseCorrelate(cv::InputArray _src1, cv::InputArray _src2,
                           cv::InputArray _window, double* response = NULL);
void createHanningWindow(cv::OutputArray _dst, cv::Size winSize, int type);

} // namespace phasecorrelation
