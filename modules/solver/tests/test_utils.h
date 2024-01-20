#pragma once

#include <cmath>
#include <initializer_list>
#include <string>
#include <vector>

namespace tl {

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
