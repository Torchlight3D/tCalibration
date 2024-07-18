#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types_c.h>

/// Compute color image gradient
void ComputeGradientMapByPrewitt(IplImage *smoothImg, short *gradImg,
                                 unsigned char *dirImg, int GRADIENT_THRESH);
