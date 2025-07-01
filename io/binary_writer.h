
#include <iostream>
#include <ostream>

#include "io/binary_utils.h"
#include "utils/logging.h"

namespace dyn_delta_approx {
namespace io {
template <typename T>
void writeBin(T t, std::ostream& ostream) {
  ostream.write(reinterpret_cast<const char*>(&t), sizeof(T));
}
template <class Graph>
void writeBinaryUndirectedGraph(Graph& g, std::ostream& ostream, bool verbose = false) {
  TIMED_FUNC(fesf);
  auto edge_count = g.edge_count();
  auto vertex_count = g.vertex_count();
  uint64_t size_info = sizeof(edge_count);
  writeBin(size_info, ostream);
  writeBin(vertex_count, ostream);
  writeBin(edge_count, ostream);
  for (auto e : g.edges()) {
    auto [u, v] = g.edge(e);
    writeBin(u, ostream);
    writeBin(v, ostream);
    writeBin(g.edgeWeight(e), ostream);
    if (verbose && e % 100000 == 0) {
      std::cout << e << " edges written." << std::endl;
    }
  }
  for (auto v : g.nodes()) {
    writeBin(g.nodeWeight(v), ostream);
    if (verbose && v % 100000 == 0) {
      std::cout << v << " nodes written." << std::endl;
    }
  }
}
}  // namespace io
}  // namespace dyn_delta_approx