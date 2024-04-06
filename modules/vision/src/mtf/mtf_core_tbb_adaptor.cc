#include "mtf_core_tbb_adaptor.h"

#include "mtf_core.h"
#include "point_helpers.h"
#include "stride_range.h"

Mtf_core_tbb_adaptor::Mtf_core_tbb_adaptor(Mtf_core *core) : mtf_core(core) {}

void Mtf_core_tbb_adaptor::operator()(const Stride_range &r) const
{
    for (size_t i = r.begin(); i != r.end(); r.increment(i)) {
        auto it = mtf_core->cl.get_boundaries().find(mtf_core->valid_obj[i]);
        cv::Point2d cent = centroid(it->second);
        mtf_core->search_borders(cent, mtf_core->valid_obj[i]);
    }
}
