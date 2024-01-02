#pragma once

#include <vector>

#include "types.h"

namespace vrender {

class VRenderParams;

// Implements some global optimizations on the polygon sorting.
class Optimizer
{
public:
    virtual ~Optimizer() {}

    virtual void optimize(std::vector<PrimitivePtr> &,
                          VRenderParams &params) = 0;
};

//  Optimizes visibility by culling primitives which do not appear in the
// rendered image. Computations are done analytically rather than using an item
// buffer.
class VisibilityOptimizer : public Optimizer
{
public:
    void optimize(std::vector<PrimitivePtr> &, VRenderParams &params) override;
};

//  Optimizes by collapsing together primitives which can be, without
// perturbating the back to front painting algorithm.
class PrimitiveSplitOptimizer : public Optimizer
{
public:
    void optimize(std::vector<PrimitivePtr> &, VRenderParams &params) override
    {
    }
};

class BackFaceCullingOptimizer : public Optimizer
{
public:
    void optimize(std::vector<PrimitivePtr> &, VRenderParams &params) override;
};

} // namespace vrender
