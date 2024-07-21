#pragma once

namespace tl {

class Distribution
{
public:
    Distribution() = default;
    virtual ~Distribution() = default;

    virtual double Eval(double x) const = 0;
};

class NormalDistribution : public Distribution
{
public:
    NormalDistribution(double mean, double sigma);

    double Eval(double x) const override;

private:
    double alpha_;
    double beta_;
    double mean_;
};

class UniformDistribution : public Distribution
{
public:
    UniformDistribution(double left, double right);

    double Eval(double x) const override;

protected:
    const double left_;
    const double right_;
    double inverse_span_;
};

} // namespace tl
