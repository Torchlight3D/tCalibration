#pragma once

#include <cmath>

#include <tMath/RANSAC/ModelEstimator>

namespace tl {

struct Point
{
    double x;
    double y;

    constexpr Point() : Point(0., 0.) {}
    constexpr Point(double x, double y) : x(x), y(y) {}
};

// y = mx + b
struct Line
{
    double m;
    double b;

    // default: y = 0
    constexpr Line() : Line(0., 0.) {}
    constexpr Line(double m, double b) : m(m), b(b) {}
};

class LineEstimator : public Estimator<Point, Line>
{
public:
    LineEstimator() {}
    ~LineEstimator() {}

    int SampleSize() const override { return 2; }

    bool EstimateModel(const std::vector<Point> &data,
                       std::vector<Line> *models) const override
    {
        Line model;
        model.m = (data[1].y - data[0].y) / (data[1].x - data[0].x);
        model.b = data[1].y - model.m * data[1].x;
        models->push_back(model);
        return true;
    }

    double Error(const Point &point, const Line &line) const override
    {
        double a = -1.0 * line.m;
        double b = 1.0;
        double c = -1.0 * line.b;
        return std::fabs(a * point.x + b * point.y + c) /
               std::sqrt(a * a + b * b);
    }
};

class Polynome
{
public:
    explicit Polynome(const std::vector<double> &coefficients);
    Polynome(const double *coefficients, size_t degree);
    Polynome(std::initializer_list<double> coefficients);

    double operator()(double x);

    size_t degree() const { return _degree; }

private:
    // coefficients must be in increasing order !
    std::vector<double> _coefficients;
    size_t _degree;
};

inline Polynome::Polynome(const std::vector<double> &coefficients)
    : _coefficients(coefficients), _degree(coefficients.size() - 1)
{
}

inline Polynome::Polynome(const double *coefficients, size_t degree)
    : Polynome(std::vector<double>(coefficients, coefficients + degree + 1))
{
}

inline Polynome::Polynome(std::initializer_list<double> coefficients)
    : Polynome(std::vector<double>{coefficients.begin(), coefficients.end()})
{
}

inline double Polynome::operator()(double x)
{
    double value{0.};
    int n{0};
    for (int i = _coefficients.size() - 1; i >= 0; i--) {
        value += _coefficients[i] * std::pow(x, n);
        n++;
    }

    return value;
}

} // namespace tl
