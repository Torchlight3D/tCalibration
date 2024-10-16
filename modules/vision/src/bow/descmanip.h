#pragma once

#include <vector>
#include <string>

#include <opencv2/core.hpp>

namespace tl {

namespace desc {

/**
 * Calculates the mean value of a set of descriptors
 * @param descriptors
 * @param mean mean descriptor
 */
void meanValue(const std::vector<cv::Mat> &descriptors, cv::Mat &mean);

/**
 * Calculates the distance between two descriptors
 * @param a
 * @param b
 * @return distance
 */
double distance(const cv::Mat &a, const cv::Mat &b);
uint32_t distance_8uc1(const cv::Mat &a, const cv::Mat &b);

/**
 * Returns a string version of the descriptor
 * @param a descriptor
 * @return string version
 */
std::string toString(const cv::Mat &a);

/**
 * Returns a descriptor from a string
 * @param a descriptor
 * @param s string version
 */
void fromString(cv::Mat &a, const std::string &s);

/**
 * Returns a mat with the descriptors in float format
 * @param descriptors
 * @param mat (out) NxL 32F matrix
 */
void toMat32F(const std::vector<cv::Mat> &descriptors, cv::Mat &mat);

/**io routines*/
void toStream(const cv::Mat &m, std::ostream &str);
void fromStream(cv::Mat &m, std::istream &str);

/**Returns the number of bytes of the descriptor
 * used for binary descriptors only*/
static size_t getDescSizeBytes(const cv::Mat &d)
{
    return d.cols * d.elemSize();
}

} // namespace desc
} // namespace tl
