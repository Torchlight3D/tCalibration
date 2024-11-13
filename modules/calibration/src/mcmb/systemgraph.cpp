#include "systemgraph.h"

#include <stack>

#include <glog/logging.h>

#include <graaflib/algorithm/shortest_path/dijkstra_shortest_path.h>
#include <graaflib/algorithm/strongly_connected_components/common.h>

namespace graaf::algorithm {

namespace {

template <typename Graph_t>
void dfs(vertex_id_t vertex_id, const Graph_t& graph,
         std::unordered_set<vertex_id_t>& seen_vertices,
         std::vector<vertex_id_t>& components)
{
    std::stack<vertex_id_t> vertex_stack;
    vertex_stack.push(vertex_id);

    while (!vertex_stack.empty()) {
        auto node = vertex_stack.top();
        vertex_stack.pop();

        if (seen_vertices.contains(node)) {
            continue;
        }

        seen_vertices.emplace(node);
        components.push_back(node);

        for (const auto& neighbor : graph.get_neighbors(node)) {
            if (!seen_vertices.contains(neighbor)) {
                vertex_stack.push(neighbor);
            }
        }
    }
}

} // namespace

template <typename Graph_t>
sccs_t connected_components(const Graph_t& graph)
{
    std::unordered_set<vertex_id_t> seen_vertices;
    seen_vertices.reserve(graph.vertex_count());

    sccs_t ccs;
    for (const auto& [vid, _] : graph.get_vertices()) {
        if (seen_vertices.contains(vid)) {
            continue;
        }

        std::vector<vertex_id_t> components;
        dfs(vid, graph, seen_vertices, components);
        ccs.push_back(components);
    }

    return ccs;
}

} // namespace graaf::algorithm

namespace tl::mcmb {

void Graph::addVertex(int id)
{
    if (_idToIndex.contains(id)) {
        return;
    }

    _idToIndex.emplace(id, _g.add_vertex(id));
}

void Graph::addEdge(int id1, int id2, double weight)
{
    addVertex(id1);
    addVertex(id2);

    _g.add_edge(_idToIndex.at(id1), _idToIndex.at(id2), weight);
}

size_t Graph::vertexCount() const { return _g.vertex_count(); }

size_t Graph::edgeCount() const { return _g.edge_count(); }

std::vector<std::vector<int>> Graph::connectedComponents() const
{
    const auto ccs_indices = graaf::algorithm::connected_components(_g);

    // Return actual node values
    std::vector<std::vector<int>> ccs;
    ccs.reserve(ccs_indices.size());
    for (const auto& cc_indices : ccs_indices) {
        std::vector<int> nodes;
        nodes.reserve(cc_indices.size());
        for (const auto& c_index : cc_indices) {
            nodes.push_back(_g.get_vertex(c_index));
        }
        ccs.push_back(nodes);
    }

    return ccs;
}

std::vector<int> Graph::shortestPathBetween(int id1, int id2) const
{
    if (!_idToIndex.contains(id1) || !_idToIndex.contains(id2)) {
        return {};
    }

    const auto shortestPath = graaf::algorithm::dijkstra_shortest_path(
        _g, _idToIndex.at(id1), _idToIndex.at(id2));
    if (!shortestPath.has_value()) {
        return {};
    }

    const auto& vertices = shortestPath.value().vertices;

    // Return actual node values
    std::vector<int> nodes;
    nodes.reserve(vertices.size());
    for (const auto& v : vertices) {
        nodes.push_back(_g.get_vertex(v));
    }

    return nodes;
}

} // namespace tl::mcmb
