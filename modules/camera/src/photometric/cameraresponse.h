#pragma once

#include <array>
#include <cassert>
#include <vector>

namespace tl {

class CameraResponse
{
public:
    CameraResponse() = default;
    virtual ~CameraResponse() = default;

    virtual void setParameters(const std::vector<double>& params) = 0;
    virtual const double* parameters() const = 0;
    virtual double* rParameters() = 0;
    virtual std::vector<double> asVector() const = 0;
    virtual size_t numParameters() const = 0;

    // Evaluates the inverse-response for the given pixel intensity.
    virtual double operator()(double value) const = 0;

    // Resets the model parameters, which results in a linear response.
    virtual void reset() = 0;

    // Returns the modeled pixel intensity range. These values can be used to
    // adapt the current model, when processing images with different formats.
    inline std::pair<double, double> range() const { return range_; }

    inline void setRange(double min, double max)
    {
        if (max < min) {
            std::swap(min, max);
        }

        range_ = {min, max};
    }

    // Determines if given intensity value is within value response range
    inline bool inRange(double value) const
    {
        return value >= range_.first && value <= range_.second;
    }

protected:
    std::pair<double, double> range_;
};

template <typename Derived, size_t ParameterCount>
class CameraResponse_ : public CameraResponse
{
public:
    CameraResponse_() : CameraResponse() { init(); }
    virtual ~CameraResponse_() = default;

    void setParameters(const std::vector<double>& params) final
    {
        assert(params.size() == params_.size());
        std::copy_n(params.cbegin(), kNumParameters, params_.begin());
    }

    const double* parameters() const final { return params_.data(); }

    double* rParameters() final { return params_.data(); }

    std::vector<double> asVector() const final
    {
        return {params_.cbegin(), params_.cend()};
    }

    inline static constexpr auto kNumParameters = ParameterCount;
    size_t numParameters() const final { return kNumParameters; }

    double operator()(double value) const final
    {
        assert(inRange(value));
        return Derived::evaluate(params_.data(), value);
    }

    void reset() final { Derived::resetParameters(params_.data()); }

protected:
    std::array<double, kNumParameters> params_;

private:
    void init()
    {
        range_ = {0., 1.};
        Derived::resetParameters(params_.data());
    }
};

} // namespace tl
