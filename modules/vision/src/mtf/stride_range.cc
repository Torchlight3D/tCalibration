#include "stride_range.h"

Stride_range::Stride_range(size_t start, size_t end, size_t stride)
    : first(start), last(end), stride(stride)
{
}

size_t Stride_range::begin() const { return first; }

size_t Stride_range::end() const
{
    return last + 2 * stride; // excluded end value
}

size_t& Stride_range::increment(size_t& v) const
{
    if (v == last) {
        v = last + 2 * stride;
        return v;
    }

    if (v + stride > last) {
        v = last;
        return v;
    }

    v += stride;
    return v;
}
