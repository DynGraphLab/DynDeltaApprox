#pragma once

#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "app/algorithms/dyn_algorithm_impl.h"
namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation {
template <class DynGraph>
class StrongEdgeOrientation {
  using NT = typename DynGraph::NodeType;
  std::vector<std::pair<int, int>> m_parent;
  std::vector<NT> m_depth;
  std::vector<int> m_out_degree;  // out degree
  // First is the neighbor id, second is the offset in its m_adj subvector
  std::vector<std::vector<std::pair<NT, NT>>> m_adj;

  inline void flip_active_edge(NT u, NT e) {
    if (m_out_degree[u] <= e) {
      throw;
    }
    // checkall();
    // edge oriented u-> v
    auto v = m_adj[u][e].first;
    auto in_v_offset = m_adj[u][e].second;
    // if (in_v_offset < m_out_degree[v]) {
    //   throw;
    // }
    // assert(in_v_offset >= _nodes_out[v]);

    auto& outflow_u = m_out_degree[u];
    // assert(outflow_u > 0);
    auto& outflow_v = m_out_degree[v];

    // move v  in u part to end
    NT last_in_u_array_active = outflow_u - 1;
    auto last_in_u_array_target = m_adj[u][last_in_u_array_active].first;
    NT last_in_u_array_target_reverse = m_adj[u][last_in_u_array_active].second;
    // assert(flat_edge_info[_nodes_offset[last_in_u_array_target] +
    //                       InsideArrayRefType{last_in_u_array_target_reverse}]
    //                       ==
    //        u);
    m_adj[u][e].first = last_in_u_array_target;
    m_adj[u][e].second = last_in_u_array_target_reverse;
    // UPDATE_SECOND(u, e, last_in_u_array_target_reverse)

    if (e != last_in_u_array_active) {
      m_adj[last_in_u_array_target][last_in_u_array_target_reverse].second = e;
      // UPDATE_SECOND(last_in_u_array_target, last_in_u_array_target_reverse, e)
    }

    auto rev_e = in_v_offset;
    // assert(rev_e < flat_edge_info.size());
    // assert(flat_edge_info[rev_e] == u);
    // assert(flat_edge_info_rel_reverse[rev_e] ==
    //        IndexType{e - _nodes_offset[u]});
    // assert(rev_e >= _nodes_offset[v] + _nodes_out[v]);
    // // now write
    // assert(in_v_offset >= _nodes_out[v]);
    NT last_in_v_inactive = outflow_v;
    NT last_in_v_target = m_adj[v][last_in_v_inactive].first;
    NT last_in_v_target_reverse = m_adj[v][last_in_v_inactive].second;

    m_adj[v][rev_e].first = last_in_v_target;
    m_adj[v][rev_e].second = last_in_v_target_reverse;

    if (rev_e != last_in_v_inactive) {
      m_adj[last_in_v_target][last_in_v_target_reverse].second = in_v_offset;
    }
    // now write u
    m_adj[v][last_in_v_inactive].first = u;

    // m_adj[v][last_in_v_inactive].second = outflow_u - 1;
    m_adj[v][last_in_v_inactive].second = outflow_u - 1;

    // now write v into the last position
    m_adj[u][last_in_u_array_active].first = v;
    m_adj[u][last_in_u_array_active].second = outflow_v;
    // UPDATE_SECOND(u, last_in_u_array_active, outflow_v)

    // assert(last_in_u_array_target == flat_edge_info[e]);
    outflow_v++;
    outflow_u--;
    // checkall();
  }
  void flip_reverse_edge(NT u, NT offset) {
    flip_active_edge(m_adj[u][offset].first, m_adj[u][offset].second);
  }
  int64_t maxOutDegree() const {
    int64_t m_max_degree = 0;
    for (const auto adj : m_out_degree) {
      m_max_degree = std::max((int64_t)adj, m_max_degree);
    }
    return m_max_degree;
  }
  double squaredSumDegree() const {
    double result = 0;
    for (const auto adj : m_out_degree) {
      result += std::pow((double)adj, 2);
    }
    return result;
  }
  int bfs_max_depth = 20;
  void bfs_backward(NT source) {
    std::vector<NT> touched;
    std::queue<NT> bfsqueue;
    int m_max_degree = m_out_degree[source] + 1;

    bfsqueue.push(source);
    m_parent[source] = {source, 0};
    m_depth[source] = 0;
    touched.push_back(source);

    int cur_depth = 0;
    bool found_lower_deg_vertex = false;
    NT final_vertex;

    while (!bfsqueue.empty()) {
      NT node = bfsqueue.front();
      bfsqueue.pop();

      if (m_depth[node] == cur_depth) {
        cur_depth++;
      }
      unsigned out_d = m_out_degree[node];
      for (unsigned i = out_d; i < m_adj[node].size(); i++) {
        NT target = m_adj[node][i].first;
        if (m_depth[target] == -1 && m_out_degree[target] >= m_max_degree) {
          m_depth[target] = cur_depth;
          bfsqueue.push(target);
          m_parent[target] = {node, i};
          touched.push_back(target);
          if (m_out_degree[target] > m_max_degree) {
            found_lower_deg_vertex = true;
            final_vertex = target;
            break;
          }
        }
      }
      if (found_lower_deg_vertex) {
        break;
      }
    }

    if (found_lower_deg_vertex) {
      // follow path and swap it
      NT curNode = final_vertex;
      std::vector<std::pair<NT, NT>> path;
      while (m_parent[curNode].first != curNode) {
        NT parent = m_parent[curNode].first;
        path.push_back({parent, m_parent[curNode].second});
        curNode = m_parent[curNode].first;
      };
      for (auto i = path.begin(); i != path.end(); i++) {
        flip_reverse_edge(i->first, i->second);
      }
    }

    for (NT v : touched) {
      m_parent[v] = {0, 0};
      m_depth[v] = -1;
    }
  }
  void bfs_forward(NT source) {
    std::vector<NT> touched;
    std::queue<NT> bfsqueue;
    int m_max_degree = m_out_degree[source] - 1;

    bfsqueue.push(source);
    m_parent[source] = {source, 0};
    m_depth[source] = 0;
    touched.push_back(source);

    int cur_depth = 0;
    bool found_lower_deg_vertex = false;
    NT final_vertex;

    while (!bfsqueue.empty()) {
      NT node = bfsqueue.front();
      bfsqueue.pop();
      // if (m_out_degree[node] < m_max_degree) {
      //   found_lower_deg_vertex = true;
      //   final_vertex = node;
      //   break;
      // }
      if (m_depth[node] == cur_depth) {
        cur_depth++;
      }
      unsigned out_d = m_out_degree[node];
      for (unsigned i = 0; i < out_d; i++) {
        NT target = m_adj[node][i].first;
        if (m_depth[target] == -1 && m_out_degree[target] <= m_max_degree) {
          m_depth[target] = cur_depth;
          bfsqueue.push(target);
          m_parent[target] = {node, i};
          touched.push_back(target);
          if (m_out_degree[target] < m_max_degree) {
            found_lower_deg_vertex = true;
            final_vertex = target;
            break;
          }
        }
      }
      if (found_lower_deg_vertex) {
        break;
      }
    }

    if (found_lower_deg_vertex) {
      // follow path and swap it
      NT curNode = final_vertex;
      std::vector<std::tuple<NT, NT, NT>> res;
      while (m_parent[curNode].first != curNode) {
        NT parent = m_parent[curNode].first;
        res.push_back({parent, m_parent[curNode].second, curNode});
        curNode = m_parent[curNode].first;
      };
      for (auto i = res.rbegin(); i != res.rend(); i++) {
        // std::cout << i - res.rbegin() << std::endl;
        // std::cout << std::get<0>(*i) << " "
        //           << m_adj[std::get<0>(*i)][std::get<1>(*i)].first << " "
        //           << std::get<2>(*i) << std::endl;
        flip_active_edge(std::get<0>(*i), std::get<1>(*i));
      }
    }

    for (NT v : touched) {
      m_parent[v] = {0, 0};
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
  StrongEdgeOrientation(const DynGraph& g, MapType<StringType, double> double_params,
                        MapType<StringType, int64_t> int64_params)
      : graph(g.currentNumEdges(), g.currentNumNodes()),
        bfs_max_depth(int64_params["bfs_depth"] ? int64_params["bfs_depth"] : 20) {
    m_out_degree.resize(g.currentNumNodes(), 0);
    m_parent.resize(g.currentNumNodes(), {0, 0});
    m_depth.resize(g.currentNumNodes(), -1);
    m_adj.resize(g.currentNumNodes());
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
    if (m_out_degree[source] > m_out_degree[target]) {
      std::swap(target, source);
    }
    // std::cout << m_out_degree[source] << " " << m_out_degree[target] <<
    // std::endl;
    //  W.l.o.g: ODEG(source) <= ODEG(target)
    //  first insert in target (easy)
    m_adj[target].push_back({source, m_out_degree[source]});
    // inserting in source array
    m_adj[source].push_back({target, m_adj[target].size() - 1});
    if (m_out_degree[source] != m_adj[source].size() - 1) {
      std::swap(m_adj[source].back(), m_adj[source][m_out_degree[source]]);
      auto [t, ref_t] = m_adj[source].back();
      // m_adj[t][ref_t].second = m_adj[source].size() - 1;
      m_adj[t][ref_t].second = m_adj[source].size() - 1;
    }
    m_out_degree[source]++;

    // checkdegrees();
    //  std::cout << "BFS" << std::endl;
    bfs_forward(source);
  }
  void handle_deletion(const typename DynGraph::StreamMember& edge) {
    auto source = edge.first;
    auto target = edge.second;
    NT decremented;
    NT ref_target = 0;
    bool gound = false;
    // First find the target in source's list and update
    for (unsigned i = 0; i < m_adj[source].size(); i++) {
      if (m_adj[source][i].first == target) {
        gound = true;
        // This ref_target is used needed to find the other entry later.
        ref_target = m_adj[source][i].second;
        // We now swap out the value
        if (i < m_out_degree[source]) {  // by first swapping with the
                                         // out_degree-1 one
          std::swap(m_adj[source][i], m_adj[source][m_out_degree[source] - 1]);
          auto [new_p, ref] = m_adj[source][i];
          m_adj[new_p][ref].second = i;
          // for outer loop set
          i = m_out_degree[source] - 1;
          m_out_degree[source]--;
          decremented = source;
        }
        // last swap
        std::swap(m_adj[source][i], m_adj[source][m_adj[source].size() - 1]);
        if (i != m_adj[source].size() - 1) {  // update only needed when unequal
          auto [new_p, ref] = m_adj[source][i];
          m_adj[new_p][ref].second = i;
        }
        m_adj[source].pop_back();
        break;
      }
    }
    assert(m_adj[target][ref_target].first == source);
    // Secondly use the pointer to find other entry.
    if (ref_target < m_out_degree[target]) {
      if (m_out_degree[target] - 1 != ref_target) {
        std::swap(m_adj[target][ref_target], m_adj[target][m_out_degree[target] - 1]);
        auto [new_p, ref] = m_adj[target][ref_target];
        // m_adj[new_p][ref].second = ref_target;

        m_adj[new_p][ref].second = ref_target;

        ref_target = m_out_degree[target] - 1;
      }
      m_out_degree[target]--;
      decremented = target;
    }
    // last swap
    std::swap(m_adj[target][ref_target], m_adj[target][m_adj[target].size() - 1]);
    if (ref_target != m_adj[target].size() - 1) {  // update only needed when unequal
      auto [new_p, ref] = m_adj[target][ref_target];
      //  m_adj[new_p][ref].second = ref_target;
      // UPDATE_SECOND(new_p, ref, ref_target)
      m_adj[new_p][ref].second = ref_target;
    }
    m_adj[target].pop_back();
    // int o = checkmaxdegree('d');
    // TODO: Update Routine.
    bfs_backward(decremented);
  }
};
}  // namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation