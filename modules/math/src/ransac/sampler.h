#pragma once

#include <memory>
#include <vector>

namespace tl {

class RandomNumberGenerator;

class Sampler
{
public:
    explicit Sampler(size_t minNumSamples,
                     std::shared_ptr<RandomNumberGenerator> rng = {});
    virtual ~Sampler();

    // Initializes any non-trivial variables and sets up sampler if
    // necessary. Must be called before Sample is called.
    virtual bool Initialize(size_t dataSize) = 0;

    // Samples the input variable data and fills the vector subset with the
    // samples.
    virtual bool Sample(std::vector<size_t>* indices) = 0;

protected:
    std::shared_ptr<RandomNumberGenerator> _rng;
    size_t _minNumSamples;
};

// Brief:
// Default Sampler for Ransac. This is guaranteed to generate a unique sample by
// performing a Fisher-Yates sampling.
class RandomSampler final : public Sampler
{
public:
    using Sampler::Sampler;

    bool Initialize(size_t dataSize) override;

    bool Sample(std::vector<size_t>* indices) override;

private:
    std::vector<size_t> _indices;
};

} // namespace tl
