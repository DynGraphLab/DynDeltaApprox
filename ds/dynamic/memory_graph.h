#pragma once
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
namespace dyn_delta_approx::ds::dynamic {
template <typename NT>
struct DynamicGraphMemory {
  using NodeType = NT;
  enum class Mode : bool { DELETION = 0, INSERTION = 1 };
  using StreamMember = std::pair<NodeType, NodeType>;
  static constexpr absl::string_view ds_name = "dynamic_graph_inmemory";

 private:
  size_t currentElem = 0;
  std::vector<std::pair<StreamMember, Mode>> elems;
  size_t num_nodes;

 public:
  DynamicGraphMemory(const std::vector<std::pair<StreamMember, Mode>>& graph, size_t n)
      : elems(graph), num_nodes(n) {}
  bool good() const { return currentElem < elems.size(); }
  std::pair<StreamMember, Mode> next() { return elems[currentElem++]; }
  size_t currentNumEdges() const { return elems.size(); }
  size_t currentNumNodes() const { return num_nodes; }
};
}  // namespace dyn_delta_approx::ds::dynamic
