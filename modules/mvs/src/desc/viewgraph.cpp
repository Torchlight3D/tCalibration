#include "viewgraph.h"

#include <ceres/rotation.h>

#include <tCore/ContainerUtils>
#include <tCore/HashUtils>

#include "../graph/connectedcomponents.h"
#include "../graph/minimumspanningtree.h"

namespace tl {

using Eigen::Matrix3d;
using Eigen::Vector3d;

bool ViewGraph::removeView(ViewId viewId)
{
    const auto* neighborIds = con::FindOrNull(vertices_, viewId);
    if (!neighborIds) {
        return false;
    }

    // Remove the edges to the view from adjacent vertices.
    for (const auto& neighborId : *neighborIds) {
        vertices_[neighborId].erase(viewId);
        const ViewIdPair viewIdPair{viewId, neighborId};
        edges_.erase(viewIdPair);
    }

    // Remove the view as a vertex.
    vertices_.erase(viewId);
    return true;
}

std::unordered_set<ViewId> ViewGraph::viewIds() const
{
    std::unordered_set<ViewId> viewIds;
    viewIds.reserve(vertices_.size());
    for (const auto& [viewId, _] : vertices_) {
        viewIds.insert(viewId);
    }
    return viewIds;
}

const std::unordered_set<ViewId>* ViewGraph::neighborViewIds(
    ViewId viewId) const
{
    return con::FindOrNull(vertices_, viewId);
}

size_t ViewGraph::numViews() const { return vertices_.size(); }

bool ViewGraph::hasView(ViewId id) const { return vertices_.contains(id); }

void ViewGraph::addEdge(ViewId id1, ViewId id2, const ViewPairInfo& info)
{
    if (id1 == id2) {
        DLOG(WARNING) << "Failed to add edge to view graph: "
                         "Not allowed to add an edge from any view to itself.";
        return;
    }

    const ViewIdPair viewPairId{id1, id2};

    DLOG_IF(WARNING, edges_.contains(viewPairId))
        << "An edge already exists between view " << id1 << " and view " << id2
        << ". "
           "\n "
           "Info of the edge will be updated to new one.";

    vertices_[id1].insert(id2);
    vertices_[id2].insert(id1);
    edges_[viewPairId] = info;
}

bool ViewGraph::removeEdge(ViewId id1, ViewId id2)
{
    const ViewIdPair viewIdPair{id1, id2};
    if (!edges_.contains(viewIdPair)) {
        return false;
    }

    // Erase the edge from each vertex.
    return vertices_[id1].erase(id2) == 1 && vertices_[id2].erase(id1) == 1 &&
           edges_.erase(viewIdPair) == 1;
}

std::vector<ViewId> ViewGraph::removeIsolatedEdges()
{
    // FIXME: Duplicated code in
    // Extract all connected components.
    ConnectedComponents<ViewId> cc_extractor;
    for (const auto& [id, _] : edges_) {
        cc_extractor.AddEdge(id.first, id.second);
    }

    std::unordered_map<ViewId, std::unordered_set<ViewId>> connected_components;
    cc_extractor.Extract(&connected_components);

    // Find the largest connected component.
    int max_cc_size = 0;
    ViewId largest_cc_root_id = kInvalidViewId;
    for (const auto& [id, neighborIds] : connected_components) {
        if (neighborIds.size() > max_cc_size) {
            max_cc_size = neighborIds.size();
            largest_cc_root_id = id;
        }
    }

    // Remove all view pairs containing a view to remove (i.e. the ones that are
    // not in the largest connectedcomponent).
    const auto numViewPairsBefore = numViewPairs();

    std::vector<ViewId> removedViewIds;
    for (const auto& [id, neighborIds] : connected_components) {
        if (id == largest_cc_root_id) {
            continue;
        }

        // NOTE: The connected component will contain the root id as well, so we
        // do not explicity have to remove connected_component.first since it
        // will exist in connected_components.second
        for (const auto& viewId2 : neighborIds) {
            removeView(viewId2);
            removedViewIds.push_back(viewId2);
        }
    }

    const auto numRemovedViewPairs = numViewPairsBefore - numViewPairs();
    LOG_IF(INFO, numRemovedViewPairs > 0)
        << numRemovedViewPairs
        << " view pairs are removed, as they are disconnected from the largest "
           "connected component of the view graph.";

    return removedViewIds;
}

const ViewPairInfo* ViewGraph::edge(ViewId id1, ViewId id2) const
{
    return con::FindOrNull(edges_, ViewIdPair{id1, id2});
}

ViewPairInfo* ViewGraph::rEdge(ViewId id1, ViewId id2)
{
    return con::FindOrNull(edges_, ViewIdPair{id1, id2});
}

const std::unordered_map<ViewIdPair, ViewPairInfo>& ViewGraph::edgesAndInfo()
    const
{
    return edges_;
}

size_t ViewGraph::numEdges() const { return edges_.size(); }

bool ViewGraph::hasEdge(ViewIdPair edge) const { return edges_.contains(edge); }

void ViewGraph::ExtractSubgraph(const std::unordered_set<ViewId>& viewIds,
                                ViewGraph* subgraph) const
{
    CHECK_NOTNULL(subgraph);

    // Iterate over each vertex.
    for (const auto& [vertex, neighbors] : vertices_) {
        if (!viewIds.contains(vertex)) {
            continue;
        }

        // Iterate over all edges to the current vertex.
        for (const auto& neighbor : neighbors) {
            // Skip this vertex (edge), if
            // 1. It is not in the subgraph.
            // 2. vertex1 >= vertex2 so as not to add redundant to the subgraph.
            if (!viewIds.contains(neighbor) || neighbor < vertex) {
                continue;
            }

            // Add the edge to the subgraph.
            const auto& edge =
                con::FindOrDieNoPrint(edges_, ViewIdPair{vertex, neighbor});
            subgraph->addEdge(vertex, neighbor, edge);
        }
    }
}

void ViewGraph::GetLargestConnectedComponentIds(
    std::unordered_set<ViewId>* largest_cc) const
{
    // Add all edges to the connected components extractor.
    ConnectedComponents<ViewId> cc_extractor;
    for (const auto& [id, _] : edges_) {
        cc_extractor.AddEdge(id.first, id.second);
    }

    // Extract all connected components.
    std::unordered_map<ViewId, std::unordered_set<ViewId>> connected_components;
    cc_extractor.Extract(&connected_components);

    // Search for the largest CC in the viewing graph.
    auto largest_cc_id = kInvalidViewId;
    size_t largest_cc_size{0};
    for (const auto& [viewId, neighborIds] : connected_components) {
        if (neighborIds.size() > largest_cc_size) {
            largest_cc_size = neighborIds.size();
            largest_cc_id = viewId;
        }
    }

    CHECK_NE(largest_cc_id, kInvalidViewId);

    std::swap(*largest_cc, connected_components[largest_cc_id]);
}

namespace {
using HeapElement = std::pair<ViewPairInfo, ViewIdPair>;

bool SortHeapElement(const HeapElement& h1, const HeapElement& h2)
{
    return h1.first.num_verified_matches > h2.first.num_verified_matches;
}

// Computes the orientation of the neighbor camera based on the orientation of
// the source camera and the relative rotation between the cameras.
Eigen::Vector3d ComputeOrientation(const Eigen::Vector3d& srcOrientation,
                                   const ViewPairInfo& viewPairInfo,
                                   ViewId srcViewId, ViewId neighborViewId)
{
    Matrix3d source_rotation_mat, relative_rotation;
    ceres::AngleAxisToRotationMatrix(
        srcOrientation.data(),
        ceres::ColumnMajorAdapter3x3(source_rotation_mat.data()));
    ceres::AngleAxisToRotationMatrix(
        viewPairInfo.rotation.data(),
        ceres::ColumnMajorAdapter3x3(relative_rotation.data()));

    const Matrix3d neighbor_orientation =
        (srcViewId < neighborViewId)
            ? (relative_rotation * source_rotation_mat).eval()
            : (relative_rotation.transpose() * source_rotation_mat).eval();

    Vector3d orientation;
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(neighbor_orientation.data()),
        orientation.data());
    return orientation;
}

// Adds all the edges of view_id to the heap. Only edges that do not already
// have an orientation estimation are added.
void AddEdgesToHeap(
    const ViewGraph& viewGraph,
    const std::unordered_map<ViewId, Eigen::Vector3d>& orientations,
    ViewId viewId, std::vector<HeapElement>* heap)
{
    const std::unordered_set<ViewId>* neighborIds =
        viewGraph.neighborViewIds(viewId);
    for (const auto& neighborId : *neighborIds) {
        // Only add edges to the heap that contain a vertex that has not been
        // seen.
        if (orientations.contains(neighborId)) {
            continue;
        }

        heap->emplace_back(*viewGraph.edge(viewId, neighborId),
                           ViewIdPair{viewId, neighborId, false});
        std::push_heap(heap->begin(), heap->end(), SortHeapElement);
    }
}

} // namespace

bool ViewGraph::OrientationsFromMaximumSpanningTree(
    std::unordered_map<ViewId, Eigen::Vector3d>* orientations) const
{
    CHECK_NOTNULL(orientations);

    // Compute the largest connected component of the input view graph since the
    // MST is only valid on a single connected component.
    std::unordered_set<ViewId> largest_cc;
    GetLargestConnectedComponentIds(&largest_cc);
    ViewGraph largest_cc_subgraph;
    ExtractSubgraph(largest_cc, &largest_cc_subgraph);

    // Compute maximum spanning tree.
    MinimumSpanningTree<ViewId, int> mst_extractor;
    for (const auto& [id, info] : largest_cc_subgraph.viewPairs()) {
        // Since we want the *maximum* spanning tree, we negate all of the edge
        // weights in the *minimum* spanning tree extractor.
        mst_extractor.AddEdge(id.first, id.second, -info.num_verified_matches);
    }

    std::unordered_set<std::pair<ViewId, ViewId>> mst;
    if (!mst_extractor.Extract(&mst)) {
        VLOG(2) << "Could not extract the maximum spanning tree from the view "
                   "graph";
        return false;
    }

    // Create an MST view graph.
    ViewGraph mstViewGraph;
    for (const auto& [viewId1, viewId2] : mst) {
        mstViewGraph.addEdge(viewId1, viewId2,
                             *largest_cc_subgraph.edge(viewId1, viewId2));
    }

    // Chain the relative rotations together to compute orientations.  We use a
    // heap to determine the next edges to add to the minimum spanning tree.
    std::vector<HeapElement> heap;

    // Set the root value.
    const auto [rootViewId, _] = *mst.cbegin();
    (*orientations)[rootViewId] = Vector3d::Zero();
    AddEdgesToHeap(mstViewGraph, *orientations, rootViewId, &heap);

    while (!heap.empty()) {
        // Copy
        const auto [nextInfo, nextId] = heap.front();

        // Remove the best edge
        std::pop_heap(heap.begin(), heap.end(), SortHeapElement);
        heap.pop_back();

        // If the edge contains two vertices that have already been added then
        // do nothing.
        if (orientations->contains(nextId.second)) {
            continue;
        }

        // Compute the orientation for the vertex.
        (*orientations)[nextId.second] =
            ComputeOrientation(con::FindOrDie(*orientations, nextId.first),
                               nextInfo, nextId.first, nextId.second);

        // Add all edges to the heap.
        AddEdgesToHeap(mstViewGraph, *orientations, nextId.second, &heap);
    }

    return true;
}

} // namespace tl
