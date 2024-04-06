#pragma once

#include "ca_core.h"
#include "stride_range.h"

class Ca_core_tbb_adaptor
{
public:
    Ca_core_tbb_adaptor(Ca_core& core) : ca_core(core) {}

    void operator()(const Stride_range& r) const
    {
        for (size_t i = r.begin(); i != r.end(); r.increment(i)) {
            ca_core.calculate_ca(ca_core.mtf_core.get_blocks()[i]);
        }
    }

    Ca_core& ca_core;
};
