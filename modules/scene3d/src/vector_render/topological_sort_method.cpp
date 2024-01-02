#include "sort_method.h"

#include <climits>

#include "axis_aligned_box.h"
#include "primitive.h"
#include "primitive_positioning.h"
#include "vector2.h"
#include "vrender_params.h"

// #define DEBUG_TS

namespace vrender {

namespace {

void checkAndAddEdgeToGraph(size_t a, size_t b,
                            std::vector<std::vector<size_t>>& precedence_graph)
{
    auto& graph = precedence_graph[a];
    bool found = std::any_of(graph.begin(), graph.end(),
                             [&b](const auto e) { return e == b; });
    if (!found) {
        graph.push_back(b);
    }
}

void recursFindNeighbors(const std::vector<PrimitivePtr>& primitive_tab,
                         const std::vector<size_t>& pindices,
                         std::vector<std::vector<size_t>>& precedence_graph,
                         const AxisAlignedBox2D& bbox, int depth)
{
    constexpr size_t MAX_PRIMITIVES_IN_CELL = 5;

    // Refinment: first decide which sub-cell each primitive meets, then call
    // algorithm recursively.
    if (primitive_tab.size() > MAX_PRIMITIVES_IN_CELL) {
        std::vector<size_t> p_indices_min_min;
        std::vector<size_t> p_indices_min_max;
        std::vector<size_t> p_indices_max_min;
        std::vector<size_t> p_indices_max_max;

        double xmin = bbox.mini().x();
        double ymin = bbox.mini().y();
        double xmax = bbox.maxi().x();
        double ymax = bbox.maxi().y();

        double xMean = 0.5 * (xmin + xmax);
        double yMean = 0.5 * (ymin + ymax);

        for (const auto& idx : pindices) {
            bool left = primitive_tab[idx]->bbox().mini().x() <= xMean;
            bool right = primitive_tab[idx]->bbox().maxi().x() >= xMean;
            bool down = primitive_tab[idx]->bbox().mini().y() <= yMean;
            bool up = primitive_tab[idx]->bbox().maxi().y() >= yMean;

            if (left && down)
                p_indices_min_min.push_back(idx);
            if (right && down)
                p_indices_max_min.push_back(idx);
            if (left && up)
                p_indices_min_max.push_back(idx);
            if (right && up)
                p_indices_max_max.push_back(idx);
        }

        // checks if refining is not too much stupid

        if (p_indices_min_min.size() < pindices.size() &&
            p_indices_max_min.size() < pindices.size() &&
            p_indices_min_max.size() < pindices.size() &&
            p_indices_max_max.size() < pindices.size()) {
            recursFindNeighbors(
                primitive_tab, p_indices_min_min, precedence_graph,
                AxisAlignedBox2D({xmin, xMean}, {ymin, yMean}), depth + 1);
            recursFindNeighbors(
                primitive_tab, p_indices_min_max, precedence_graph,
                AxisAlignedBox2D({xmin, xMean}, {yMean, ymax}), depth + 1);
            recursFindNeighbors(
                primitive_tab, p_indices_max_min, precedence_graph,
                AxisAlignedBox2D({xMean, xmax}, {ymin, yMean}), depth + 1);
            recursFindNeighbors(
                primitive_tab, p_indices_max_max, precedence_graph,
                AxisAlignedBox2D({xMean, xmax}, {yMean, ymax}), depth + 1);
            return;
        }
    }

    // No refinment either because it could not be possible, or because the
    // number of primitives is below the predefined limit.

    for (size_t i = 0; i < pindices.size(); ++i)
        for (size_t j = i + 1; j < pindices.size(); ++j) {
            // Compute the position of j as regard to i

            int prp = PrimitivePositioning::computeRelativePosition(
                primitive_tab[pindices[i]], primitive_tab[pindices[j]]);

            if (prp & PrimitivePositioning::Upper)
                checkAndAddEdgeToGraph(pindices[j], pindices[i],
                                       precedence_graph);
            if (prp & PrimitivePositioning::Lower)
                checkAndAddEdgeToGraph(pindices[i], pindices[j],
                                       precedence_graph);
        }
}

void buildPrecedenceGraph(std::vector<PrimitivePtr>& primitive_tab,
                          std::vector<std::vector<size_t>>& precedence_graph)
{
    // The precedence graph is constructed by first conservatively determining
    // which primitives can possibly intersect using a quadtree. Candidate pairs
    // of primitives are then carefully checked to compute their exact relative
    // positionning.
    //
    // Because of the conservativeness of the quadtree, some pairs of primitives
    // may be checked multiple times for intersection. Using a buffer of already
    // computed results may proove very efficient.

    // 0 - compute bounding box of the set of primitives.
    AxisAlignedBox2D BBox;
    for (const auto& primitive : primitive_tab) {
        const auto primitiveBBox = primitive->bbox();
        BBox.include({primitiveBBox.mini().x(), primitiveBBox.mini().y()});
        BBox.include({primitiveBBox.maxi().x(), primitiveBBox.maxi().y()});
    }

    // 1 - recursively find pairs.
    std::vector<size_t> pindices(primitive_tab.size());
    for (size_t j = 0; j < pindices.size(); ++j) {
        pindices[j] = j;
    }

    recursFindNeighbors(primitive_tab, pindices, precedence_graph, BBox, 0);
}

void suppressPrecedence(size_t a, size_t b,
                        std::vector<std::vector<size_t>>& precedence_graph)
{
    std::vector<size_t> prec_tab = std::vector<size_t>(precedence_graph[a]);

    // FIXME: wtf???
    bool trouve = false;
    for (size_t k = 0; k < prec_tab.size(); ++k) {
        if (prec_tab[k] == b) {
            prec_tab[k] = prec_tab[prec_tab.size() - 1];
            prec_tab.pop_back();
        }
    }

    if (!trouve) {
        throw std::runtime_error("Unexpected error in suppressPrecedence");
    }
}

void recursTopologicalSort(std::vector<std::vector<size_t>>& precedence_graph,
                           std::vector<PrimitivePtr>& primitive_tab,
                           std::vector<bool>& already_rendered,
                           std::vector<bool>& already_visited,
                           std::vector<PrimitivePtr>& new_pr_tab, size_t indx,
                           size_t& nb_cycles, VRenderParams& vparams,
                           size_t info_cnt, size_t& nbrendered)
{
    // One must first render the primitives indicated by the precedence graph,
    // then render the current primitive. Skews are detected, but and treated.

    already_visited[indx] = true;

    for (size_t j = 0; j < precedence_graph[indx].size(); ++j) {
        // Both tests are important. If we ommit the second one, the recursion
        // is always performed down to the next cycle, although this is useless
        // if the current primitive was rendered already.

        if (!already_visited[precedence_graph[indx][j]]) {
            if (!already_rendered[precedence_graph[indx][j]]) {
                recursTopologicalSort(precedence_graph, primitive_tab,
                                      already_rendered, already_visited,
                                      new_pr_tab, precedence_graph[indx][j],
                                      nb_cycles, vparams, info_cnt, nbrendered);
            }
        }
        else {
            //  A cycle is detected, but in this version, it is not broken.
            ++nb_cycles;
        }
    }

    if (!already_rendered[indx]) {
        new_pr_tab.push_back(primitive_tab[indx]);

        if ((++nbrendered) % info_cnt == 0) {
            vparams.progress(nbrendered / (float)primitive_tab.size(),
                             "Topological sort");
        }
    }

    already_rendered[indx] = true;
    already_visited[indx] = false;
}

void recursTopologicalSort(std::vector<std::vector<size_t>>& precedence_graph,
                           std::vector<PrimitivePtr>& primitive_tab,
                           std::vector<bool>& already_rendered,
                           std::vector<bool>& already_visited,
                           std::vector<PrimitivePtr>& new_pr_tab, size_t indx,
                           std::vector<size_t>& ancestors,
                           size_t& ancestors_backward_index, size_t& nb_cycles,
                           VRenderParams& vparams, size_t info_cnt,
                           size_t& nbrendered)
{
    // One must first render the primitives indicated by the precedence graph,
    // then render the current primitive. Skews are detected, but and treated.

    already_visited[indx] = true;
    ancestors.push_back(indx);

    for (size_t j = 0; j < precedence_graph[indx].size(); ++j) {
        // Both tests are important. If we ommit the second one, the recursion
        // is always performed down to the next cycle, although this is useless
        // if the current primitive was rendered already.

        if (!already_visited[precedence_graph[indx][j]]) {
            if (!already_rendered[precedence_graph[indx][j]]) {
                recursTopologicalSort(precedence_graph, primitive_tab,
                                      already_rendered, already_visited,
                                      new_pr_tab, precedence_graph[indx][j],
                                      ancestors, ancestors_backward_index,
                                      nb_cycles, vparams, info_cnt, nbrendered);

                if (ancestors_backward_index != INT_MAX &&
                    ancestors.size() > (size_t)(ancestors_backward_index + 1)) {
#ifdef DEBUG_TS
                    cout << "Returning early" << endl;
#endif
                    already_visited[indx] = false;
                    ancestors.pop_back();
                    return;
                }
                if (ancestors_backward_index !=
                    INT_MAX) // we are returning from emergency. j must be
                    // re-tried
                    --j;
            }
        }
        else {
            //  A cycle is detected. It must be broken. The algorithm is the
            //  following: primitives of the cycle
            // are successively split by a chosen splitting plane and the
            // precendence graph is updated at the same time by re-computing
            // primitive precedence. As soon as the cycle is broken, the
            // algorithm stops and calls recursively calls on the new precedence
            // graph. This necessarily happens because of the BSP-node nature of
            // the current set of primitives.

            // 0 - stats
            ++nb_cycles;

            // 0.5 - determine cycle beginning

            long cycle_beginning_index = -1;
            for (size_t i = ancestors.size() - 1;
                 long(i) >= 0 && cycle_beginning_index < 0; --i)
                if (ancestors[i] == precedence_graph[indx][j])
                    cycle_beginning_index = (long)i;
#ifdef DEBUG_TS
            cout << "Unbreaking cycle : ";
            for (size_t i = 0; i < ancestors.size(); ++i)
                cout << ancestors[i] << " ";
            cout << precedence_graph[indx][j] << endl;
#endif
#ifdef DEBUG_TS
            assert(cycle_beginning_index >= 0);
#endif
            // 1 - determine splitting plane

            long split_prim_ancestor_indx = -1;
            long split_prim_indx = -1;

            // Go down ancestors tab, starting from the skewing primitive, and
            // stopping at it.

            for (size_t i2 = (size_t)cycle_beginning_index;
                 i2 < ancestors.size() && split_prim_ancestor_indx < 0; ++i2)
                if (primitive_tab[ancestors[i2]]->nbVertices() > 2) {
                    split_prim_ancestor_indx = (long)i2;
                    split_prim_indx = (long)ancestors[i2];
                }

#ifdef DEBUG_TS
            cout << "Split primitive index = " << split_prim_ancestor_indx
                 << "(primitive = " << split_prim_indx << ")" << endl;
#endif
            if (split_prim_indx <
                0) // no need to unskew cycles between segments and points
                continue;

            // 2 - split all necessary primitives

            const auto* P = dynamic_cast<const Polygone*>(
                primitive_tab[(size_t)split_prim_indx]);
            const NVector3& normal = NVector3(P->normal());
            double c(P->c());
            ancestors.push_back(precedence_graph[indx][j]); // sentinel
            ancestors.push_back(
                ancestors[(size_t)cycle_beginning_index + 1]); // sentinel
            bool cycle_broken = false;

            for (size_t i3 = (size_t)cycle_beginning_index + 1;
                 i3 < ancestors.size() - 1 && !cycle_broken; ++i3)
                if (ancestors[i3] != (size_t)split_prim_indx) {
                    bool prim_lower_ante_contains_im1 = false;
                    bool prim_upper_ante_contains_im1 = false;
                    bool prim_lower_prec_contains_ip1 = false;
                    bool prim_upper_prec_contains_ip1 = false;

                    Primitive* prim_upper = nullptr;
                    Primitive* prim_lower = nullptr;

                    PrimitivePositioning::splitPrimitive(
                        primitive_tab[ancestors[i3]], normal, c, prim_upper,
                        prim_lower);

                    if (!prim_upper || !prim_lower)
                        continue;
#ifdef DEBUG_TS
                    cout << "Splitted primitive " << ancestors[i3] << endl;
#endif

                    std::vector<size_t> prim_upper_prec;
                    std::vector<size_t> prim_lower_prec;

                    std::vector<size_t> old_prec =
                        std::vector<size_t>(precedence_graph[ancestors[i3]]);

                    size_t upper_indx = precedence_graph.size();
                    size_t lower_indx = ancestors[i3];

                    //  Updates the precedence graph downwards.

                    for (size_t k = 0; k < old_prec.size(); ++k) {
                        int prp1 =
                            PrimitivePositioning::computeRelativePosition(
                                prim_upper, primitive_tab[old_prec[k]]);
#ifdef DEBUG_TS
                        cout << "Compariing " << upper_indx << " and "
                             << old_prec[k] << ": ";
#endif
                        // It can not be Upper, because it was lower from the \
    // original primitive, but it is not necessary lower any \
    // longer because of the split.

                        if (prp1 & PrimitivePositioning::Lower) {
#ifdef DEBUG_TS
                            cout << " > " << endl;
#endif
                            prim_upper_prec.push_back(old_prec[k]);

                            if (old_prec[k] == ancestors[i3 + 1])
                                prim_upper_prec_contains_ip1 = true;
                        }
#ifdef DEBUG_TS
                        else
                            cout << " I " << endl;
#endif

                        int prp2 =
                            PrimitivePositioning::computeRelativePosition(
                                prim_lower, primitive_tab[old_prec[k]]);
#ifdef DEBUG_TS
                        cout << "Compariing " << lower_indx << " and "
                             << old_prec[k] << ": ";
#endif
                        if (prp2 & PrimitivePositioning::Lower) {
#ifdef DEBUG_TS
                            cout << " > " << endl;
#endif
                            prim_lower_prec.push_back(old_prec[k]);

                            if (old_prec[k] == ancestors[i3 + 1])
                                prim_lower_prec_contains_ip1 = true;
                        }
#ifdef DEBUG_TS
                        else
                            cout << " I " << endl;
#endif
                    }

                    // We also have to update the primitives which are upper to
                    // the current one, because some of them may not be upper
                    // anymore. This would requires either a O(n^2) algorithm,
                    // or to store an dual precedence graph. For now it's
                    // O(n^2). This test can not be skipped because upper can
                    // still be lower to ancestors[i-1].

                    for (size_t l = 0; l < precedence_graph.size(); ++l)
                        if (l != (size_t)lower_indx)
                            for (size_t k = 0; k < precedence_graph[l].size();
                                 ++k)
                                if (precedence_graph[l][k] == ancestors[i3]) {
                                    int prp1 = PrimitivePositioning::
                                        computeRelativePosition(
                                            prim_upper, primitive_tab[l]);

                                    // It can not be Lower, because it was upper
                                    // from the original primitive, but it is
                                    // not necessary upper any longer because of
                                    // the split.

                                    if (prp1 & PrimitivePositioning::Upper) {
                                        // Still upper. Add the new index at end
                                        // of the array

                                        precedence_graph[l].push_back(
                                            upper_indx);

                                        if (l == (size_t)ancestors[i3 - 1])
                                            prim_upper_ante_contains_im1 = true;
                                    }
                                    // If the primitive is not upper anymore
                                    // there is nothing to change since the
                                    // index has changed.

                                    int prp2 = PrimitivePositioning::
                                        computeRelativePosition(
                                            prim_lower, primitive_tab[l]);
#ifdef DEBUG_TS
                                    cout << "Compariing " << l << " and "
                                         << lower_indx << ": ";
#endif
                                    if (prp2 & PrimitivePositioning::Upper) {
#ifdef DEBUG_TS
                                        cout << " > " << endl;
#endif
                                        if (l ==
                                            (size_t)
                                                ancestors[i3 -
                                                          1]) // The index is
                                                              // the same =>
                                            // nothing to change.
                                            prim_lower_ante_contains_im1 = true;
                                    }
                                    else {
#ifdef DEBUG_TS
                                        cout << " I " << endl;
#endif
                                        // Not upper anymore. We have to \
    // suppress this entry from the tab.

                                        precedence_graph[l][k] =
                                            precedence_graph
                                                [l][precedence_graph[l].size() -
                                                    1];
                                        precedence_graph[l].pop_back();
                                        --k;
                                    }

                                    break; // each entry is present only once.
                                }

                    // setup recorded new info

                    primitive_tab.push_back(prim_upper);
                    delete primitive_tab[lower_indx];
                    primitive_tab[lower_indx] = prim_lower;

                    // Adds the info to the precedence graph

                    precedence_graph.push_back(prim_upper_prec);
                    precedence_graph[lower_indx] = prim_lower_prec;

                    // Adds new entries to the 'already_rendered' and
                    // 'already_visited' vectors
                    already_visited.push_back(false);
                    already_rendered.push_back(false);
#ifdef DEBUG_TS
                    cout << "New precedence graph: " << endl;
                    printPrecedenceGraph(precedence_graph, primitive_tab);
#endif
                    // Checks if the cycle is broken. Because the graph is only \
    // updated downwards, we check wether lower (which still is \
    // lower to ancestors[i-1]) is upper to ancestors[i+1], or \
    // if upper is still .

                    if ((!(prim_lower_ante_contains_im1 &&
                           prim_lower_prec_contains_ip1)) &&
                        (!(prim_upper_ante_contains_im1 &&
                           prim_upper_prec_contains_ip1)))
                        cycle_broken = true;
                }

            ancestors.pop_back();
            ancestors.pop_back();

            // 3 - recurs call

            if (cycle_broken) {
                ancestors_backward_index = (size_t)cycle_beginning_index;
#ifdef DEBUG_TS
                cout << "Cycle broken. Jumping back to rank "
                     << ancestors_backward_index << endl;
#endif
                already_visited[indx] = false;
                ancestors.pop_back();
                return;
            }
#ifdef DEBUG_TS
            else
                cout << "Cycle could not be broken !!" << endl;
#endif
        }
    }

    if (!already_rendered[indx]) {
#ifdef DEBUG_TS
        cout << "Returning ok. Rendered primitive " << indx << endl;
#endif
        new_pr_tab.push_back(primitive_tab[indx]);

        if ((++nbrendered) % info_cnt == 0)
            vparams.progress(nbrendered / (float)primitive_tab.size(),
                             "Advanced topological sort");
    }

    already_rendered[indx] = true;
    ancestors_backward_index = INT_MAX;
    already_visited[indx] = false;
    ancestors.pop_back();
}

void topologicalSort(std::vector<std::vector<size_t>>& precedence_graph,
                     std::vector<PrimitivePtr>& primitive_tab,
                     VRenderParams& vparams)
{
    std::vector<PrimitivePtr> new_pr_tab;
    std::vector<bool> already_visited(primitive_tab.size(), false);
    std::vector<bool> already_rendered(primitive_tab.size(), false);
    size_t nb_skews = 0;

    size_t info_cnt = primitive_tab.size() / 200 + 1;
    size_t nbrendered = 0;

    // 1 - sorts primitives by rendering order

    for (size_t i = 0; i < primitive_tab.size(); ++i)
        if (!already_rendered[i])
            recursTopologicalSort(precedence_graph, primitive_tab,
                                  already_rendered, already_visited, new_pr_tab,
                                  i, nb_skews, vparams, info_cnt, nbrendered);

#ifdef DEBUG_TS
    if (nb_skews > 0)
        cout << nb_skews << " cycles found." << endl;
    else
        cout << "No cycles found." << endl;
#endif
    primitive_tab = new_pr_tab;
}

void topologicalSortBreakCycles(
    std::vector<std::vector<size_t>>& precedence_graph,
    std::vector<PrimitivePtr>& primitive_tab, VRenderParams& vparams)
{
    std::vector<PrimitivePtr> new_pr_tab;
    std::vector<bool> already_visited(primitive_tab.size(), false);
    std::vector<bool> already_rendered(primitive_tab.size(), false);
    std::vector<size_t> ancestors;
    size_t nb_skews = 0;
    size_t ancestors_backward_index;

    size_t info_cnt = primitive_tab.size() / 200 + 1;
    size_t nbrendered = 0;

    // 1 - sorts primitives by rendering order

    for (size_t i = 0; i < primitive_tab.size(); ++i)
        if (!already_rendered[i])
            recursTopologicalSort(precedence_graph, primitive_tab,
                                  already_rendered, already_visited, new_pr_tab,
                                  i, ancestors, ancestors_backward_index,
                                  nb_skews, vparams, info_cnt, nbrendered);

#ifdef DEBUG_TS
    if (nb_skews > 0)
        cout << nb_skews << " cycles found." << endl;
    else
        cout << "No cycles found." << endl;
#endif
    primitive_tab = new_pr_tab;
}

#ifdef DEBUG_TS
static void printPrecedenceGraph(
    const std::vector<std::vector<size_t>>& precedence_graph,
    const std::vector<PtrPrimitive>& primitive_tab);
#endif
}; // namespace

void TopologicalSortMethod::sortPrimitives(
    std::vector<PrimitivePtr>& primitive_tab, VRenderParams& vparams)
{
    // 1 - build a precedence graph

#ifdef DEBUG_TS
    cout << "Computing precedence graph." << endl;
    cout << "Old order: ";
    for (size_t i = 0; i < primitive_tab.size(); ++i)
        cout << (void*)(primitive_tab[i]) << " ";
    cout << endl;
#endif
    std::vector<std::vector<size_t>> precedence_graph(primitive_tab.size());
    buildPrecedenceGraph(primitive_tab, precedence_graph);

#ifdef DEBUG_TS
    TopologicalSortUtils::printPrecedenceGraph(precedence_graph, primitive_tab);
#endif
    // 2 - perform a topological sorting of the graph

#ifdef DEBUG_TS
    cout << "Sorting." << endl;
#endif

    if (_break_cycles)
        topologicalSortBreakCycles(precedence_graph, primitive_tab, vparams);
    else
        topologicalSort(precedence_graph, primitive_tab, vparams);

#ifdef DEBUG_TS
    cout << "New order: ";
    for (size_t i = 0; i < primitive_tab.size(); ++i)
        cout << (void*)(primitive_tab[i]) << " ";
    cout << endl;
#endif
}

#ifdef DEBUG_TS
void TopologicalSortUtils::printPrecedenceGraph(
    const vector<vector<size_t>>& precedence_graph,
    const vector<PtrPrimitive>& primitive_tab)
{
    for (size_t i = 0; i < precedence_graph.size(); ++i) {
        cout << i << " (" << primitive_tab[i]->nbVertices() << ") : ";
        for (size_t j = 0; j < precedence_graph[i].size(); ++j)
            cout << precedence_graph[i][j] << " ";

        cout << endl;
    }
}
#endif

} // namespace vrender
