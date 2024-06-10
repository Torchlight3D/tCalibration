#pragma once

#include <cassert>
#include <vector>

namespace tl {

class Vignetting
{
public:
    Vignetting(int width, int height) : width_(width), height_(height) {}
    virtual ~Vignetting() = default;

    inline int width() const { return width_; }
    inline int height() const { return height_; }

    inline void setParameters(const std::vector<double>& params)
    {
        assert(params.size() == params_.size());
        params_ = params;
    }
    inline const std::vector<double>& asVector() const { return params_; }
    inline size_t numParameters() const { return params_.size(); }

    // Evaluates the attenuation at the specified point in the image.
    virtual double operator()(double u, double v) const = 0;

    // Resets the model parameters, which results in uniform attenuation.
    virtual void reset() = 0;

protected:
    int width_, height_;
    std::vector<double> params_;
};

template <typename Derived>
class Vignetting_ : public Vignetting
{
public:
    Vignetting_(int width, int height) : Vignetting(width, height) { init(); }
    virtual ~Vignetting_() = default;

    double operator()(double u, double v) const final
    {
        return Derived::evaluate(params_.data(), width_, height_, u, v);
    }

    void reset() final
    {
        Derived::resetParameters(params_.data(), width_, height_);
    }

private:
    void init()
    {
        params_.resize(Derived::getNumParameters(width_, height_));
        reset();
    }
};

} // namespace tl
