#pragma once

#include <opencv2/imgproc.hpp>

#include <sstream>

namespace tl {
namespace runetag {

struct DefGreenText
{
    static const unsigned char col_r = 0;
    static const unsigned char col_g = 255;
    static const unsigned char col_b = 0;
    static const int thickness = 1;
    static const int scale = 5;
    static const int x = 10;
    static const int y = 20;
    static const bool drop_shadow = true;
};

static struct endl_type
{
} newline;

template <typename DefSettings = DefGreenText>
class ImgTextStream
{
public:
    ImgTextStream(cv::Mat& _frame)
        : frame(_frame),
          fontScale(static_cast<double>(DefSettings::scale) / 10.0),
          color(CV_RGB(DefSettings::col_r, DefSettings::col_g,
                       DefSettings::col_b)),
          thickness(DefSettings::thickness),
          x(static_cast<float>(DefSettings::x)),
          base_x(static_cast<float>(DefSettings::x)),
          y(static_cast<float>(DefSettings::y)),
          drop_shadow(DefSettings::drop_shadow) {};

    class position
    {
    public:
        position(float _x, float _y) : x(_x), y(_y) {}
        position(cv::Point2f p) : x(p.x), y(p.y) {}
        float x;
        float y;
    };

    class color
    {
    public:
        color(unsigned char _r, unsigned char _g, unsigned char _b)
            : r(_r), g(_g), b(_b)
        {
        }
        unsigned char r;
        unsigned char g;
        unsigned char b;
    };

    class thickness
    {
    public:
        thickness(int _t) : t(_t) {}
        int t;
    };

    class dropshadow
    {
        dropshadow(bool _enable) : enable(_enable) {}
        bool enable;
    };

    friend ImgTextStream& operator<<(ImgTextStream& _ts, const position& _pos)
    {
        _ts.x = _ts.base_x = _pos.x;
        _ts.y = _pos.y;
        return _ts;
    }

    friend ImgTextStream& operator<<(ImgTextStream& _ts, const color& _col)
    {
        _ts.color = CV_RGB(_col.r, _col.g, _col.b);
        return _ts;
    }

    friend ImgTextStream& operator<<(ImgTextStream& _ts, double _scale)
    {
        _ts.fontScale = _scale;
        return _ts;
    }

    friend ImgTextStream& operator<<(ImgTextStream& _ts,
                                     const thickness& _thickness)
    {
        _ts.thickness = _thickness.t;
        return _ts;
    }

    friend ImgTextStream& operator<<(ImgTextStream& _ts, endl_type e)
    {
        cv::Size textSize =
            cv::getTextSize(std::string("M"), cv::FONT_HERSHEY_SIMPLEX,
                            _ts.fontScale, _ts.thickness, nullptr);
        _ts.x = _ts.base_x;
        _ts.y += textSize.height + 3;
        return _ts;
    }

    template <typename T>
    friend ImgTextStream& operator<<(ImgTextStream& _ts, T _v)
    {
        std::stringstream ss(std::stringstream::out);
        ss << _v;

        if (_ts.drop_shadow) {
            cv::putText(_ts.frame, ss.str(), cv::Point2f(_ts.x + 1, _ts.y + 1),
                        cv::FONT_HERSHEY_SIMPLEX, _ts.fontScale,
                        CV_RGB(50, 50, 50), _ts.thickness + 2);
        }

        cv::putText(_ts.frame, ss.str(), cv::Point2f(_ts.x, _ts.y),
                    cv::FONT_HERSHEY_SIMPLEX, _ts.fontScale, _ts.color,
                    _ts.thickness);

        const auto textSize =
            cv::getTextSize(ss.str(), cv::FONT_HERSHEY_SIMPLEX, _ts.fontScale,
                            _ts.thickness, 0);
        _ts.x += textSize.width;

        return _ts;
    }

private:
    cv::Mat& frame;
    double fontScale;
    cv::Scalar color;
    int thickness;
    float x;
    float base_x;
    float y;
    bool drop_shadow;
};

typedef ImgTextStream<> its;

} // namespace runetag
} // namespace tl
