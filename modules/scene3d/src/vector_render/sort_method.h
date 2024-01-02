#pragma once

#include <vector>

#include "types.h"

namespace vrender {

class VRenderParams;

class SortMethod
{
public:
    SortMethod() {}
    virtual ~SortMethod() {}

    virtual void sortPrimitives(std::vector<PrimitivePtr> &primitives,
                                VRenderParams &params) = 0;

    void SetZDepth(FLOAT s) { zSize = s; }
    FLOAT ZDepth() const { return zSize; }

protected:
    FLOAT zSize;
};

class DontSortMethod : public SortMethod
{
public:
    using SortMethod::SortMethod;

    void sortPrimitives(std::vector<PrimitivePtr> &primitives,
                        VRenderParams &params) override
    {
    }
};

class BSPSortMethod : public SortMethod
{
public:
    using SortMethod::SortMethod;

    void sortPrimitives(std::vector<PrimitivePtr> &primitives,
                        VRenderParams &params) override;
};

class TopologicalSortMethod : public SortMethod
{
public:
    using SortMethod::SortMethod;

    void sortPrimitives(std::vector<PrimitivePtr> &primitives,
                        VRenderParams &params) override;

    void setBreakCycles(bool b) { _break_cycles = b; }

private:
    bool _break_cycles{false};
};

} // namespace vrender
