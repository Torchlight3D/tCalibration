#pragma once

#include <unordered_map>
#include <unordered_set>

#include "viewpairinfo.h"

namespace tl {

// Brief:
// An undirected graph containing the views relationship in a Scene.
// Internally, the graph is holding all the nodes (Views) to their connected
// nodes (Views), and all the edges (ViewPairs) to their infos.
class ViewGraph
{
public:
    ViewGraph() = default;

    /// Views
    // Remove view and all the edges connected to the view from the graph
    bool removeView(ViewId id);

    std::unordered_set<ViewId> viewIds() const;
    const std::unordered_set<ViewId>* neighborViewIds(ViewId id) const;
    inline auto connectedViewIds(ViewId id) const
    {
        return neighborViewIds(id);
    }
    size_t numViews() const;
    bool hasView(ViewId id) const;

    /// Edges
    // Add an edge between the two views with the their corresponding info.
    // New vertices are added to the graph if they did not already exist. If an
    // edge already existed between the two views then the edge value is
    // updated.
    void addEdge(ViewId id1, ViewId id2, const ViewPairInfo& info);
    inline auto addViewPair(ViewId id1, ViewId id2, const ViewPairInfo& info)
    {
        return addEdge(id1, id2, info);
    }
    bool removeEdge(ViewId id1, ViewId id2);
    inline auto removeViewPair(ViewId id1, ViewId id2)
    {
        return removeEdge(id1, id2);
    }
    std::vector<ViewId> removeIsolatedEdges();
    inline auto removeDisconnectedViewPairs() { return removeIsolatedEdges(); }

    // Each edge is indexed by the ViewPairId(ViewId_1, ViewId_2) such that
    // ViewId_1 < ViewId_2.
    const ViewPairInfo* edge(ViewId id1, ViewId id2) const;
    ViewPairInfo* rEdge(ViewId id1, ViewId id2);
    inline auto viewPair(ViewId id1, ViewId id2) const
    {
        return edge(id1, id2);
    }
    inline auto rViewPair(ViewId id1, ViewId id2) { return rEdge(id1, id2); }
    const std::unordered_map<ViewIdPair, ViewPairInfo>& edgesAndInfo() const;
    inline const auto& viewPairs() const { return edgesAndInfo(); }
    size_t numEdges() const;
    inline auto numViewPairs() const { return numEdges(); }
    bool hasEdge(ViewIdPair pair) const;
    inline auto hasViewPair(ViewIdPair id) const { return hasEdge(id); }
    inline auto hasEdge(ViewId id1, ViewId id2) const
    {
        return hasEdge(ViewIdPair{id1, id2});
    }
    inline auto hasViewPair(ViewId id1, ViewId id2) const
    {
        return hasEdge(id1, id2);
    }

    /// Algos
    // Extract a subgraph from this view graph which contains only the input
    // views. Note that this means that only edges between the input views will
    // be preserved in the subgraph.
    void ExtractSubgraph(const std::unordered_set<ViewId>& viewIds,
                         ViewGraph* sub) const;

    // Returns the views ids participating in the largest connected component in
    // the view graph.
    void GetLargestConnectedComponentIds(
        std::unordered_set<ViewId>* largest_cc) const;

    // Computes orientations of each view in the view graph by computing the
    // maximum spanning tree (by edge weight) and solving for the global
    // orientations by chaining rotations. Orientations are estimated for only
    // the largest connected component of the viewing graph.
    bool OrientationsFromMaximumSpanningTree(
        std::unordered_map<ViewId, Eigen::Vector3d>* orientations) const;

private:
    std::unordered_map<ViewId, std::unordered_set<ViewId>> vertices_;
    std::unordered_map<ViewIdPair, ViewPairInfo> edges_;
};

} // namespace tl
