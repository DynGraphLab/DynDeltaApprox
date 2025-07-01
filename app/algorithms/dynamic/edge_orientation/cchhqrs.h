
#pragma once

#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "app/algorithms/dyn_algorithm_impl.h"
#include "dynamic_edge_orientation/cchhqrs/base_types.h"
#include "dynamic_edge_orientation/cchhqrs/bucket.h"

namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation {
template <class DynGraph>
class CCHHQRSEdgeOrientation {
  using NT = typename DynGraph::NodeType;
  using DEdge = dyn_delta_approx::dynamic_edge_orientation::cchhqrs::DEdge<NT>;
  using Vertex = dyn_delta_approx::dynamic_edge_orientation::cchhqrs::Vertex<
      dyn_delta_approx::dynamic_edge_orientation::cchhqrs::Buckets, NT>;

  std::vector<DEdge> edge_arena;  // TODO delete edges at the end
  size_t edge_arena_ptr = 0;
  std::vector<Vertex> vertices;
#ifdef VALIDATE_EDGE_ORIENTATION
  std::map<std::pair<int, int>, bool> seen_edges;
#endif
  int64_t maxOutDegree() const {
    int64_t outd = 0;
    int64_t sum = 0;
    int64_t i = 0;
    for (const auto& vertex : vertices) {
      int64_t v_out = 0;
      for (auto e = 0UL; e < vertex.active_edges; e++) {
        DEdge* uv = vertex.out_edges[e];
        v_out += std::floor(uv->count / b);
        if (uv->count > b) std::cout << uv->count << std::endl;

        // "partial" edges
        // if there is a half edge, we break ties arbitrarily
        if (2 * (uv->count % b) == b) {
          if (i > uv->target) {
            v_out++;
          }
        } else if (uv->count % b > b / 2) {
          v_out++;
        }
      }
      outd = std::max(outd, v_out);
      sum += v_out;
      i++;
    }
    // std::cout << sum << std::endl;
    return outd;
  }
  double squaredSumDegree() const {
    double outd = 0;
    int64_t i = 0;
    for (const auto& vertex : vertices) {
      int64_t v_out = 0;
      for (DEdge* uv : vertex.out_edges) {
        for (int j = 0; j < uv->count / b; j++) {
          v_out++;
        }
        // "partial" edges
        // if there is a half edge, we break ties arbitrarily
        if (2 * (uv->count % b) == b) {
          if (i > uv->target) {
            v_out++;
          }
        } else if (uv->count % b > b / 2) {
          v_out++;
        }
      }
      outd += std::pow((double)v_out, 2);
      i++;
    }
    return outd;
  }
  unsigned int robin_size;
  int offset, theta;
  int b;
  double lambda_precomp, lambda;

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
  CCHHQRSEdgeOrientation(const DynGraph& g, MapType<StringType, double> double_params,
                         MapType<StringType, int64_t> int64_params)
      : graph(g.currentNumEdges(), g.currentNumNodes()),
        lambda(double_params["lambda"]),
        theta(int64_params["theta"]),
        b(int64_params["b"]) {
    robin_size = std::max(10.0, 2.0 / lambda) + 1;
    lambda_precomp = 1.0 / (log(1.0 + lambda));
    offset = static_cast<int>(log(std::max(1.0f, static_cast<float>(4 * b))) * lambda_precomp);
    vertices.resize(g.currentNumNodes());
    edge_arena.resize(2 * (g.currentNumEdges()) + g.currentNumNodes());
    for (int i = 0; i < g.currentNumNodes(); i++) {
      // vertices[i].in_edges = Buckets();
      vertices[i].self_loop = &edge_arena[edge_arena_ptr++];
      vertices[i].self_loop->target = i;
      vertices[i].self_loop->mirror = vertices[i].self_loop;
      add(vertices[i].self_loop, i);
    }
  }
  int64_t weight() const { return size(); }
  int64_t size() const { return maxOutDegree(); }
  double quality() { return squaredSumDegree(); }
  int free_edges_size() { return 0; };
  const DynGraphInfo& instance() const { return graph; }
  bool valid() {
#ifdef VALIDATE_EDGE_ORIENTATION

    int i = 0;
    int min_b = ((double)b) / 2.0;
    for (const auto& vertex : vertices) {
      for (auto e = 0UL; e < vertex.active_edges; e++) {
        auto uv = vertex.out_edges[e];
        int64_t v_out = 0;
        // std::cout << i << " " << uv.target << " " << uv.count << std::endl;
        for (int j = 0; j < uv->count / b; j++) {
          v_out++;
        }
        // "partial" edges
        // if there is a half edge, we break ties arbitrarily
        if (2 * (uv->count % b) == b) {
          if (i > uv->target) {
            v_out++;
          }
        } else if (uv->count % b > b / 2) {
          v_out++;
        }
        if (v_out) {
          auto source = i;
          auto target = uv->target;
          // std::cout << source << " " << target << " seen" << std::endl;

          if (!seen_edges[std::make_pair(std::max(source, target), std::min(source, target))]) {
            return false;
          }
          seen_edges[std::make_pair(std::max(source, target), std::min(source, target))] = false;
        }
      }
      i++;
    }
    bool no_error = true;
    for (auto [k, v] : seen_edges) {
      if (v) {
        // std::cout << k.first << " " << k.second << std::endl;
        no_error = false;
      }
    }
    return no_error;
#else
    return true;
#endif
  }
  void handle_insertion(const typename DynGraph::StreamMember& edge) {
    auto source = edge.first;
    auto target = edge.second;
#ifdef VALIDATE_EDGE_ORIENTATION
    seen_edges[std::make_pair(std::max(source, target), std::min(source, target))] = true;
#endif
    if (source == target) {
      return;
    }

    // make the edges
    assert(edge_arena_ptr < edge_arena.size());
    DEdge* new_uv = &edge_arena[edge_arena_ptr++];
    new_uv->target = target;
    DEdge* new_vu = &edge_arena[edge_arena_ptr++];
    new_vu->target = source;

    new_uv->mirror = new_vu;
    new_vu->mirror = new_uv;

    for (unsigned i = 0; i < b; i++) {
      if (vertices[source].out_degree <= vertices[target].out_degree) {
        insert_directed_worst_case(new_uv, source);
      } else {
        insert_directed_worst_case(new_vu, target);
      }
      // Insert Directed is currently not adjusted for this new structure
    }
  }
  void handle_deletion(const typename DynGraph::StreamMember& edge) {
    auto source = edge.first;
    auto target = edge.second;
#ifdef VALIDATE_EDGE_ORIENTATION
    seen_edges[std::make_pair(std::max(source, target), std::min(source, target))] = false;
#endif
    if (source == target) {
      return;
    }
    DEdge* uv = NULL;
    DEdge* vu = NULL;

    for (DEdge* d : vertices[source].out_edges) {
      if (d->target == target) {
        uv = d;
        break;
      }
    }
    if (uv != NULL) {
      vu = uv->mirror;
    } else {
      for (DEdge* d : vertices[target].out_edges) {
        if (d->target == source) {
          vu = d;
          break;
        }
      }
      uv = vu->mirror;
    }
    while (uv->count + vu->count > 0) {
      if (uv->count >= vu->count) {
        delete_directed_worst_case(uv, source);
      } else {
        delete_directed_worst_case(vu, target);
      }
    }
    // remove out of arrays
    auto& v = vertices[vu->mirror->target];
    auto& u = vertices[uv->mirror->target];

    if (uv->location_out_neighbours != -1) {
      std::swap(u.out_edges[u.out_edges.size() - 1], u.out_edges[uv->location_out_neighbours]);
      u.out_edges[uv->location_out_neighbours]->location_out_neighbours =
          uv->location_out_neighbours;
      u.out_edges.pop_back();
    }
    if (vu->location_out_neighbours != -1) {
      assert(v.out_edges.size() > 0);
      assert(vu->location_out_neighbours < v.out_edges.size());
      assert(vu->location_out_neighbours >= 0);

      std::swap(v.out_edges[v.out_edges.size() - 1], v.out_edges[vu->location_out_neighbours]);
      v.out_edges[vu->location_out_neighbours]->location_out_neighbours =
          vu->location_out_neighbours;
      v.out_edges.pop_back();
    }
  }
  void add(DEdge* uv, NT u) {
    vertices[u].out_degree++;
    uv->count++;
    assert(uv->count <= b);

    if (uv->count == 1) {  // new edge
      if (uv->location_out_neighbours == -1) {
        // std::cout<<"inserting "<<(size_t)uv<<std::endl;
        uv->location_out_neighbours = vertices[u].out_edges.size();
        vertices[u].out_edges.push_back(uv);
      }
      if (vertices[u].active_edges < vertices[u].out_edges.size() - 1) {
        // std::cout<<"swapping "<<(size_t)uv<<std::endl;
        assert(vertices[u].active_edges <= uv->location_out_neighbours);
        assert(vertices[u].out_edges[uv->location_out_neighbours] == uv);
        std::swap(vertices[u].out_edges[vertices[u].active_edges],
                  vertices[u].out_edges[uv->location_out_neighbours]);
        vertices[u].out_edges[uv->location_out_neighbours]->location_out_neighbours =
            uv->location_out_neighbours;
        uv->location_out_neighbours = vertices[u].active_edges;
      }

      vertices[u].active_edges++;
      assert(vertices[u].out_edges[vertices[u].active_edges - 1] == uv);
      // insert u in the in neighbours of v
      vertices[uv->target].in_edges.add(uv, vertices[u].out_degree,
                                        vertices[uv->target].self_loop->bucket, b, lambda_precomp,
                                        offset);
    }
  }
  void insert_directed_worst_case(DEdge* uv, NT u) {
    // NOLINT(*-no-recursion)
    //   assert_edges(u);
    add(uv, u);
    //  assert_edges(u);
    int t_max = 0;
    int old_robin = vertices[u].robin;
    // Round robin
    // unsigned int out_edges_size = vertices[u].out_edges.size();
    auto max_val_loop = std::min(robin_size, vertices[u].active_edges);
    const auto out_deg = vertices[u].out_degree;
    auto& u_ = vertices[u];
    for (int t = 0; t < max_val_loop; t++) {
      if (u_.robin >= vertices[u].active_edges) {
        u_.robin = 0;
      }
      assert(u_.robin < u_.active_edges);
      DEdge* uw = u_.out_edges[u_.robin];
      // Find
      if (u_.out_degree >
          std::max((double)b, vertices[uw->target].out_degree * (1.0 + lambda) + theta)) {
        remove(uw, u);
        insert_directed_worst_case(uw->mirror, uw->target);
        u_.robin++;
        if (u_.robin >= vertices[u].active_edges) {
          u_.robin = 0;
        }
        t_max = t;
        break;
      }
      u_.robin++;  //= ( u_robin+1) % u_.out_edges.size();
    }
    if (u_.robin >= vertices[u].active_edges) {
      u_.robin = 0;
    }
    // u_.robin = u_robin;
    // Update all edges visited in the previous loop
    // if(t_max>0) std::cout<<t_max<<std::endl;
    for (int t = 0; t <= t_max; t++) {
      if (old_robin >= vertices[u].active_edges) {
        old_robin = 0;
      }
      DEdge* uw = u_.out_edges[old_robin];
      vertices[uw->target].in_edges.update(uw, u_.out_degree, b, lambda_precomp, offset);
      old_robin++;
    }
  }
  void delete_directed_worst_case(DEdge* uv, NT u) {
    remove(uv, u);
    DEdge* xu = vertices[u].in_edges.get_max();
    // if (ux->target == u) {
    //   std::cout << "ALARM" << std::endl;
    // }
    if (vertices[xu->mirror->target].out_degree >
        std::max(static_cast<double>(b), (1.0 + lambda) * vertices[u].out_degree + theta)) {
      add(xu->mirror, u);
      delete_directed_worst_case(xu, xu->mirror->target);
    } else {
      for (int t = 0; t < robin_size; t++) {
        if (vertices[u].robin >= vertices[u].active_edges) {
          vertices[u].robin = 0;
        }
        DEdge* uw = vertices[u].out_edges[vertices[u].robin];
        vertices[uw->target].in_edges.update(uw, vertices[u].out_degree, b, lambda_precomp, offset);
        vertices[u].robin++;
      }
    }
  }
  void remove(DEdge* uv, NT u) {
    // decrement the multiplicity of the edge
    assert(uv->count > 0);
    uv->count--;
    if (vertices[u].robin >= vertices[u].active_edges) {
      vertices[u].robin = 0;
    }
    int robin = vertices[u].robin;
    int current_location = uv->location_out_neighbours;

    assert(current_location < vertices[u].active_edges);
    // if the new multiplicity is 0, remove the element.
    if (uv->count == 0) {
      //  del_events++;
      if (vertices[u].out_edges.size() > 1) {
        if (current_location >= robin - 1) {
          std::swap(vertices[u].out_edges[current_location],
                    vertices[u].out_edges[vertices[u].active_edges - 1]);
          vertices[u].out_edges[current_location]->location_out_neighbours = current_location;
          if (current_location == robin - 1) {
            robin--;
          }
        } else {
          // swap robin element with the element to delete
          std::swap(vertices[u].out_edges[current_location], vertices[u].out_edges[robin]);
          vertices[u].out_edges[current_location]->location_out_neighbours = current_location;
          // swap the element to delete (at robin location) with the end
          std::swap(vertices[u].out_edges[vertices[u].active_edges - 1],
                    vertices[u].out_edges[robin]);
          vertices[u].out_edges[robin]->location_out_neighbours = robin;
          robin--;
        }
      }
      // vertices[u].out_edges.resize(vertices[u].out_edges.size() - 1);
      vertices[u].active_edges--;
      uv->location_out_neighbours = vertices[u].active_edges;  // vertices[u].out_edges.size();
      vertices[uv->target].in_edges.remove(uv);
    }
    if (uv->count < 0) {
      std::cerr << "Error: edge count below 0" << std::endl;
    }
    vertices[u].out_degree--;
    vertices[u].robin = robin;
  }
};
}  // namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation