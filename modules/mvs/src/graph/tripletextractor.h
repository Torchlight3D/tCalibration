#pragma once

#include <cstdint>
#include <set>

#include <tCore/ContainerUtils>

#include "connectedcomponents.h"

namespace tl {

// Extract all loops of size 3 (i.e., triplets) in a set of view pairs. Triplets
// are then gathered into connected components where two triplets are connected
// if the share an edge in the view pairs. NOTE: This means that a single
// connected view graph may results in multiple "connected" triplet graphs.
template <typename T>
class TripletExtractor
{
public:
    using TripletId = uint32_t;
    using TypePair = std::pair<T, T>;
    using TypeTriplet = std::tuple<T, T, T>;

    // Extracts all triplets from the view pairs (which should be edges in a
    // view graph). Triplets are grouped by connectivity, and vector represents
    // a connected triplet graph.
    bool ExtractTriplets(
        const std::unordered_set<TypePair>& edge_graph,
        std::vector<std::vector<TypeTriplet>>* connected_triplets);

private:
    // Finds all triplets in the view pairs.
    void FindTriplets(const std::unordered_set<TypePair>& edge_graph);

    // Each view triplet contains 3 view pairs, so we use the lookup map to add
    // all triplets that share one of the view pairs as neighbors in the
    // connected component analysis.
    void GetConnectedTripletGraphs(
        std::unordered_map<TripletId, std::unordered_set<TripletId>>*
            connected_triplet_graphs);

    // Store the triplet in the internal container and add the entries to the
    // edge list appropriatesly.
    void StoreTriplet(const T& a, const T& b, const T& c);

private:
    // Container for all triplets found in the view pairs.
    std::vector<TypeTriplet> triplets_;

    // We keep track of the triplets that each edge in the view pairs
    // participate in for computing the connected components.
    std::unordered_map<TypePair, std::vector<TripletId>> triplet_edges_;

    // DISALLOW_COPY_AND_ASSIGN(TripletExtractor);
};

namespace internal {

// The following logic will sort the triplet in increasing order.
template <typename T>
void SortTriplet(std::tuple<T, T, T>* tuple)
{
    auto& [a, b, c] = *tuple;
    // We know that a < b (by construction) so we only need to correctly place
    // c.
    if (c < b) {
        std::swap(b, c);
    }
    if (b < a) {
        std::swap(a, b);
    }
}
} // namespace internal

template <typename T>
bool TripletExtractor<T>::ExtractTriplets(
    const std::unordered_set<TypePair>& edge_graph,
    std::vector<std::vector<TypeTriplet>>* connected_triplets)
{
    triplets_.clear();
    triplet_edges_.clear();
    triplets_.reserve(edge_graph.size());
    triplet_edges_.reserve(edge_graph.size());

    // Find all the triplets.
    FindTriplets(edge_graph);

    // Split the triplets into connected triplet graphs.
    std::unordered_map<TripletId, std::unordered_set<TripletId>>
        connected_triplet_graphs;
    GetConnectedTripletGraphs(&connected_triplet_graphs);

    // Move the connected triplets to the output.
    connected_triplets->reserve(connected_triplet_graphs.size());
    for (const auto& [_, connectedTripletIds] : connected_triplet_graphs) {
        VLOG(2) << "Extracted a connected triplet graph of containing "
                << connectedTripletIds.size() << " triplet(s)";

        std::vector<TypeTriplet> triplets;
        triplets.reserve(connectedTripletIds.size());
        for (const auto& tripletId : connectedTripletIds) {
            triplets.emplace_back(triplets_[tripletId]);
        }
        connected_triplets->emplace_back(triplets);
    }

    // Sort the triplet connected components such that the largest is at the
    // front of the output.
    std::sort(connected_triplets->begin(), connected_triplets->end(),
              [](const auto& triplets1, const auto& triplets2) {
                  return triplets1.size() > triplets2.size();
              });

    return true;
}

// Finds all triplets in the view pairs. We find loops by examining a sorted
// list of the view ids of the edges in the view graph.
template <typename T>
void TripletExtractor<T>::FindTriplets(
    const std::unordered_set<TypePair>& edge_graph)
{
    // Create an adjacency graph.
    std::unordered_map<T, std::set<T>> adjacency_graph;
    adjacency_graph.reserve(edge_graph.size());
    for (const auto& edge : edge_graph) {
        const auto [node1, node2] = edge;

        adjacency_graph[node1].emplace(node2);
        adjacency_graph[node2].emplace(node1);
    }

    // Iterate over the adjacency graph and find all triplets.
    for (const auto& edge : edge_graph) {
        const auto& [node1, node2] = edge;

        // Find any nodes that contain edges to both of the nodes for the
        // current edge. Any such nodes form a triplet in the graph. We do this
        // efficiently using a set intersection on the list of adjacent nodes to
        // each of the nodes in the current edges.
        const auto& edges_to_node1 = con::FindOrDie(adjacency_graph, node1);
        const auto& edges_to_node2 = con::FindOrDie(adjacency_graph, node2);

        // Compute the intersection between the two adjacency lists to find
        // triplets.
        std::vector<T> node_intersection;
        node_intersection.reserve(
            std::min(edges_to_node1.size(), edges_to_node2.size()));
        std::set_intersection(edges_to_node1.begin(), edges_to_node1.end(),
                              edges_to_node2.begin(), edges_to_node2.end(),
                              std::back_inserter(node_intersection));

        // Keep track of each triplet.
        for (const auto& node : node_intersection) {
            StoreTriplet(node1, node2, node);
        }

        // Remove the edge from the adjacency graph. Since this step found all
        // triplets that contain the edge, there are no possible triplets
        // remaining that include the edge so it may be safely removed to
        // improve efficiency.
        adjacency_graph[node1].erase(node2);
        adjacency_graph[node2].erase(node1);
    }
}

template <typename T>
void TripletExtractor<T>::StoreTriplet(const T& a, const T& b, const T& c)
{
    TypeTriplet triplet{a, b, c};
    internal::SortTriplet(&triplet);
    triplets_.emplace_back(triplet);

    // Add each edge of the triplet to the edge lookup map. This will be used
    // to determine connected components.
    const TripletId triplet_id = triplets_.size() - 1;
    const auto& [node1, node2, node3] = triplet;

    // Add each edge of the triplet to the lookup map.
    triplet_edges_[TypePair{node1, node2}].emplace_back(triplet_id);
    triplet_edges_[TypePair{node1, node3}].emplace_back(triplet_id);
    triplet_edges_[TypePair{node2, node3}].emplace_back(triplet_id);
}

// Each view triplet contains 3 view pairs, so we use the lookup map to add
// all triplets that share one of the view pairs as neighbors in the connected
// component analysis.
template <typename T>
void TripletExtractor<T>::GetConnectedTripletGraphs(
    std::unordered_map<TripletId, std::unordered_set<TripletId>>*
        connected_triplet_graphs)
{
    ConnectedComponents<TripletId> triplets_cc_extractor;

    // Iterate through all edges and connect the triplets that contain the edge.
    for (const auto& [_, tripletIds] : triplet_edges_) {
        // We add a self-contained edge to a triplet if it is in isolation. This
        // ensures that the triplet is still returned in the CC analysis even
        // though it has no neighboring triplets.
        if (tripletIds.size() == 1) {
            triplets_cc_extractor.AddEdge(tripletIds[0], tripletIds[0]);
            continue;
        }

        // For connected component analysis, we simply need to link all triplets
        // containing this edge together so they will be in the same conneted
        // component. We can do this by simply chaining them together.
        for (size_t i{0}; i < tripletIds.size() - 1; i++) {
            triplets_cc_extractor.AddEdge(tripletIds[i], tripletIds[i + 1]);
        }
    }

    triplets_cc_extractor.Extract(connected_triplet_graphs);
}

} // namespace tl
