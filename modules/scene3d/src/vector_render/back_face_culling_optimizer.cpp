#include "optimizer.h"

#include <vector>

#include "primitive.h"
#include "vrender_params.h"

using namespace vrender;

// Over-simplified algorithm to check wether a polygon is front-facing or not.
// Only works for convex polygons.
void BackFaceCullingOptimizer::optimize(
    std::vector<PrimitivePtr> &primitives_tab, VRenderParams &)
{
    int nb_culled{0};
    for (auto *primitive : primitives_tab) {
        if (auto P = dynamic_cast<Polygone *>(primitive)) {
            for (size_t j{0}; j < P->nbVertices(); ++j) {
                if (((P->vertex(j + 2) - P->vertex(j + 1)) ^
                     (P->vertex(j + 1) - P->vertex(j)))
                        .z() > 0.0) {
                    delete primitive;
                    primitive = nullptr;
                    ++nb_culled;
                    break;
                }
            }
        }
    }

    // Rule out gaps. This avoids testing for null primitives later.
    int j{0};
    for (size_t k{0}; k < primitives_tab.size(); ++k) {
        if (primitives_tab[k]) {
            primitives_tab[j++] = primitives_tab[k];
        }
    }

    primitives_tab.resize(j);
#ifdef DEBUG_BFC
    cout << "Backface culling: " << nb_culled << " polygons culled." << endl;
#endif
}
