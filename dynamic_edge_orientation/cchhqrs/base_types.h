#pragma once
#include <vector>
namespace dyn_delta_approx::dynamic_edge_orientation::cchhqrs {

template <typename NT>
struct DEdge {
  DEdge* mirror;  // vu
  // Vertex *source;
  NT target;
  int count = 0;
  int bucket;  // location of the edge
  int location_in_neighbours;
  int location_out_neighbours = -1;  // location of edge in the out neighbour list of
  // source
  DEdge(NT v) { target = v; };
  DEdge() {}
};
template <template <class> class Buckets, typename NT>
struct Vertex {
  std::vector<DEdge<NT>*> out_edges;
  //  std::set<DEdge*> deleted_out_edges;
  Buckets<DEdge<NT>> in_edges;
  DEdge<NT>* self_loop;
  unsigned int out_degree = 0;  // out degree, //TODO remove
  unsigned int active_edges = 0;
  unsigned int robin = 0;
};
}  // namespace dyn_delta_approx::dynamic_edge_orientation::cchhqrs