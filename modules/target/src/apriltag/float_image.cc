#include "float_image.h"

#include <algorithm>
#include <iostream>

#include <glog/logging.h>

#include "gaussian.h"

namespace apriltags {

FloatImage::FloatImage() : FloatImage(0, 0) {}

FloatImage::FloatImage(int width, int height)
    : FloatImage(width, height, std::vector<float>(width * height))
{
}

FloatImage::FloatImage(int width, int height, const std::vector<float>& data)
    : m_width(width), m_height(height), m_data(data)
{
}

FloatImage& FloatImage::operator=(const FloatImage& other)
{
    m_width = other.m_width;
    m_height = other.m_height;

    if (m_data.size() != other.m_data.size()) {
        m_data.resize(other.m_data.size());
    }
    m_data = other.m_data;

    return *this;
}

void FloatImage::decimateAvg()
{
    int nWidth = m_width / 2;
    int nHeight = m_height / 2;

    for (int y = 0; y < nHeight; y++) {
        for (int x = 0; x < nWidth; x++) {
            m_data[y * nWidth + x] = m_data[(2 * y) * m_width + (2 * x)];
        }
    }

    m_width = nWidth;
    m_height = nHeight;
    m_data.resize(nWidth * nHeight);
}

void FloatImage::normalize()
{
    const auto maxVal = *std::max_element(m_data.begin(), m_data.end());
    const auto minVal = *std::min_element(m_data.begin(), m_data.end());
    const auto range = maxVal - minVal;
    const auto rescale = 1 / range;
    for (auto& val : m_data) {
        val = (val - minVal) * rescale;
    }
}

void FloatImage::filterFactoredCentered(const std::vector<float>& hFilter,
                                        const std::vector<float>& vFilter)
{
    // do horizontal
    std::vector<float> r(m_data);

    for (int y = 0; y < m_height; y++) {
        Gaussian::convolveSymmetricCentered(m_data, y * m_width, m_width,
                                            hFilter, r, y * m_width);
    }

    // do vertical
    std::vector<float> tmp(m_height);  // column before convolution
    std::vector<float> tmp2(m_height); // column after convolution

    for (int x = 0; x < m_width; x++) {
        // copy the column out for locality
        for (int y = 0; y < m_height; y++) {
            tmp[y] = r[y * m_width + x];
        }

        Gaussian::convolveSymmetricCentered(tmp, 0, m_height, vFilter, tmp2, 0);

        for (int y = 0; y < m_height; y++) {
            m_data[y * m_width + x] = tmp2[y];
        }
    }
}

void FloatImage::printMinMax() const
{
    LOG(INFO) << "Min: " << *min_element(m_data.begin(), m_data.end())
              << ", Max: " << *max_element(m_data.begin(), m_data.end());
    // for (int i = 0; i < getNumFloatImagePixels(); i++)
    //   std::cout << "Index[" << i << "]: " <<
    //   this->normalize().getFloatImagePixels()[i] << endl;
}

} // namespace apriltags
