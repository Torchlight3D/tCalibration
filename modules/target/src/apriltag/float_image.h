#pragma once

#include <vector>

namespace DualCoding {

using uchar = unsigned char;

template <typename T>
class Sketch;

} // namespace DualCoding

namespace apriltags {

//! Represent an image as a vector of floats in [0,1]
class FloatImage
{
public:
    FloatImage();
    FloatImage(int width, int height);
    FloatImage(int width, int height, const std::vector<float>& data);

    FloatImage& operator=(const FloatImage& other);

    void set(int x, int y, float v) { m_data[index(x, y)] = v; }
    float at(int x, int y) const { return m_data[index(x, y)]; }
    const std::vector<float>& data() const { return m_data; }
    std::vector<float>& rData() { return m_data; }

    int width() const { return m_width; }
    int height() const { return m_height; }
    inline int pixelCount() const { return width() * height(); }
    int index(int x, int y) const { return y * m_width + x; }

    //! TODO: Fix decimateAvg function. DO NOT USE!
    void decimateAvg();

    void normalize();

    void filterFactoredCentered(const std::vector<float>& fhoriz,
                                const std::vector<float>& fvert);

    template <typename T>
    void copyToSketch(DualCoding::Sketch<T>& sketch) const
    {
        for (int i{0}; i < pixelCount(); i++)
            sketch[i] = (T)(255.0f * data()[i]);
    }

    void printMinMax() const;

private:
    std::vector<float> m_data;
    int m_width, m_height;
};

} // namespace apriltags
