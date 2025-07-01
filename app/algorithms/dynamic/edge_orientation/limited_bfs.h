#pragma once
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "app/algorithms/dyn_algorithm_impl.h"
namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation {

template <class DynGraph>
class LimitedBFSEdgeOrientation {
  using NT = typename DynGraph::NodeType;
  std::vector<std::vector<NT>> m_adj;

  std::vector<int> m_parent;
  std::vector<NT> m_depth;
  std::vector<std::vector<NT>> m_degree_buckets;
  std::vector<NT> m_bucket_pos;
  std::vector<int> m_degree;
  int m_max_degree;
  int64_t maxOutDegree() const { return m_max_degree; }
  double squaredSumDegree() const {
    double result = 0;
    for (const auto& adj : m_adj) {
      result += std::pow((double)adj.size(), 2);
    }
    return result;
  }
  int bfs_max_depth = 20;
  void checkdegrees() {
    int max = 0;
    for (unsigned i = 0; i < m_adj.size(); i++) {
      if (m_degree[i] != m_adj[i].size()) {
        std::cout << "problem with degrees" << std::endl;
        exit(0);
      }
      if (m_adj[i].size() > max) {
        max = m_adj[i].size();
      }
    }
    for (unsigned i = 0; i < m_adj.size(); i++) {
      if (m_degree_buckets[m_degree[i]].size() < m_bucket_pos[i]) {
        std::cout << "size problem " << m_bucket_pos[i] << " "
                  << m_degree_buckets[m_degree[i]].size() << std::endl;
        exit(0);
      }
      if (m_degree_buckets[m_degree[i]][m_bucket_pos[i]] != i) {
        std::cout << "bucket pos error" << std::endl;
        exit(0);
      }
    }
    if (max != m_max_degree) {
      std::cout << "max degree does not match" << std::endl;
      exit(0);
    }
  };

  void move_node_to_new_bucket(NT source, int new_bucket) {
    // move vertex out of its current bucket into the larger bucket
    int bucket_pos = m_bucket_pos[source];
    int bucket_size = m_degree_buckets[m_degree[source]].size();

    std::swap(m_degree_buckets[m_degree[source]][bucket_pos],
              m_degree_buckets[m_degree[source]][bucket_size - 1]);
    m_degree_buckets[m_degree[source]].pop_back();  // source is now removed from the bucket
    m_bucket_pos[m_degree_buckets[m_degree[source]][bucket_pos]] = bucket_pos;
    m_degree_buckets[new_bucket].push_back(source);
    m_bucket_pos[source] = m_degree_buckets[new_bucket].size() - 1;

    if (new_bucket > m_max_degree) {
      m_max_degree = new_bucket;
    }

    // did we decrease the max degree??
    while (m_degree_buckets[m_max_degree].size() == 0) m_max_degree--;
  };
  void bfs(NT source) {
    std::vector<NT> touched;
    std::queue<NT> bfsqueue;
    touched.push_back(source);
    bfsqueue.push(source);
    m_parent[source] = source;
    m_depth[source] = 0;
    int cur_depth = 0;
    bool found_lower_deg_vertex = false;
    NT final_vertex;

    while (!bfsqueue.empty()) {
      NT node = bfsqueue.front();
      bfsqueue.pop();

      if (m_degree[node] < m_max_degree - 1) {
        found_lower_deg_vertex = true;
        final_vertex = node;
        break;
      }

      if (cur_depth > bfs_max_depth) {
        break;
      }

      if (m_depth[node] == cur_depth) {
        cur_depth++;
      }

      for (unsigned i = 0; i < m_adj[node].size(); i++) {
        NT target = m_adj[node][i];
        if (m_depth[target] == -1) {
          m_depth[target] = cur_depth;
          bfsqueue.push(target);
          m_parent[target] = node;
          touched.push_back(target);
        }
      }
    }

    if (found_lower_deg_vertex) {
      // follow path and swap it
      NT curNode = final_vertex;
      move_node_to_new_bucket(final_vertex, m_degree[final_vertex] + 1);
      m_degree[final_vertex]++;

      while (m_parent[curNode] != curNode) {
        NT parent = m_parent[curNode];
        // insert parent in current nodes adjacency
        m_adj[curNode].push_back(parent);
        // remove curNode from parents adjacency
        // node finding the vertex in the adjaceny does not increase complexity as we did a bfs
        // before and checked all neighbors
        for (unsigned i = 0; i < m_adj[parent].size(); i++) {
          if (m_adj[parent][i] == curNode) {
            std::swap(m_adj[parent][i], m_adj[parent][m_adj[parent].size() - 1]);
            m_adj[parent].pop_back();
            break;
          }
        }
        curNode = m_parent[curNode];
      };
      move_node_to_new_bucket(curNode, m_degree[curNode] - 1);
      m_degree[curNode]--;
    }

    for (NT v : touched) {
      m_parent[v] = 0;
      m_depth[v] = -1;
    }
  }

 public:
  struct DynGraphInfo {
    size_t num_changes = 0;
    size_t num_nodes = 0;
    DynGraphInfo(size_t nc, size_t nn) : num_changes(nc), num_nodes(nn) {}
    size_t currentNumEdges() const { return num_changes; }
    size_t currentNumNodes() const { return num_nodes; }
  };
  DynGraphInfo graph;
  template <template <typename, typename> class MapType, typename StringType>
  LimitedBFSEdgeOrientation(const DynGraph& g, MapType<StringType, double> double_params,
                            MapType<StringType, int64_t> int64_params)
      : graph(g.currentNumEdges(), g.currentNumNodes()),
        bfs_max_depth(int64_params["bfs_depth"] ? int64_params["bfs_depth"] : 20) {
    m_degree.resize(g.currentNumNodes(), 0);
    m_parent.resize(g.currentNumNodes(), 0);
    m_depth.resize(g.currentNumNodes(), -1);
    m_bucket_pos.resize(g.currentNumNodes(), 0);
    m_adj.resize(g.currentNumNodes());
    m_degree_buckets.resize(g.currentNumNodes());
    m_max_degree = 0;

    for (unsigned i = 0; i < g.currentNumNodes(); i++) {
      m_degree_buckets[0].push_back(i);
      m_bucket_pos[i] = i;  // store the bucket position of node i
    }
  }
  int64_t weight() const { return size(); }
  int64_t size() const { return maxOutDegree(); }
  double quality() { return squaredSumDegree(); }
  int free_edges_size() { return 0; };
  const DynGraphInfo& instance() const { return graph; }
  bool valid() { return true; }
  void handle_insertion(const typename DynGraph::StreamMember& edge) {
    auto source = edge.first;
    auto target = edge.second;
    m_adj[source].push_back(target);
    int old_max = m_max_degree;
    move_node_to_new_bucket(source, m_degree[source] + 1);

    m_degree[source]++;

    if (m_degree[source] < m_max_degree || m_max_degree == 1) {
      return;
    } else {
      bfs(source);
    }
  }
  void handle_deletion(const typename DynGraph::StreamMember& edge) {
    auto source = edge.first;
    auto target = edge.second;
    for (unsigned i = 0; i < m_adj[source].size(); i++) {
      if (m_adj[source][i] == target) {
        std::swap(m_adj[source][i], m_adj[source][m_adj[source].size() - 1]);
        m_adj[source].pop_back();
        move_node_to_new_bucket(source, m_degree[source] - 1);
        m_degree[source]--;

        break;
      }
    }
    for (unsigned i = 0; i < m_adj[target].size(); i++) {
      if (m_adj[target][i] == source) {
        std::swap(m_adj[target][i], m_adj[target][m_adj[target].size() - 1]);
        m_adj[target].pop_back();
        move_node_to_new_bucket(target, m_degree[target] - 1);
        m_degree[target]--;

        break;
      }
    }
  }
};
}  // namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation
