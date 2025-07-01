#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "app/algorithms/dyn_algorithm_impl.h"
#include "app/algorithms/dynamic/edge_orientation/cchhqrs.h"
#include "app/algorithms/dynamic/edge_orientation/improved_bfs.h"
#include "app/algorithms/dynamic/edge_orientation/limited_bfs.h"
#include "app/algorithms/dynamic/edge_orientation/packed_cchhqrs.h"
#include "app/algorithms/dynamic/edge_orientation/strong_bfs.h"
#include "app/app_io.pb.h"
#include "ds/dynamic/memory_graph.h"
namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation {
namespace {
using dyn_delta_approx::app::algorithms::DynamicAlgorithmImpl;
using dyn_delta_approx::app::app_io::Hypergraph;
using dyn_delta_approx::app::app_io::RunConfig;
using dyn_delta_approx::ds::dynamic::DynamicGraphMemory;
}  // namespace

template <class DynamicGraph, template <typename> class EdgeOrientation>
struct DynDelta
    : dyn_delta_approx::app::algorithms::DynamicAlgorithmImpl<DynamicGraph,
                                                              EdgeOrientation<DynamicGraph>> {
  std::unique_ptr<EdgeOrientation<DynamicGraph>> Init(
      const dyn_delta_approx::app::app_io::AlgorithmConfig& config,
      const DynamicGraph& stream) override {
    auto double_params = config.double_params();
    auto int64_params = config.int64_params();

    return std::make_unique<EdgeOrientation<DynamicGraph>>(stream, double_params, int64_params);
  }
  void Stream(const typename DynamicGraph::StreamMember& member,
              const typename DynamicGraph::Mode mode, EdgeOrientation<DynamicGraph>& solution) {
    // TIMED_FUNC(test);
    if (mode == DynamicGraph::Mode::INSERTION) {
      solution.handle_insertion(member);
    } else {
      solution.handle_deletion(member);
    }
  }
};
template <template <typename> class EdgeOrientation, typename NT = int>
struct DynDeltaInMemory : public DynDelta<DynamicGraphMemory<NT>, EdgeOrientation> {
  absl::StatusOr<std::unique_ptr<DynamicGraphMemory<NT>>> Load(
      const RunConfig& run_config, const Hypergraph& hypergraph) override {
    if (hypergraph.format() == "seq") {
      using StreamMember = typename DynamicGraphMemory<NT>::StreamMember;
      using Mode = typename DynamicGraphMemory<NT>::Mode;

      std::vector<std::pair<StreamMember, Mode>> graph;
      std::ifstream input(hypergraph.file_path());
      std::string line;
      size_t num_nodes = 0;
      size_t num_edges = 0;
      // int undirected = 0;

      std::getline(input, line);
      std::stringstream first_line(line);

      std::string hash;
      first_line >> hash;
      if (hash != "#") {
        return absl::InvalidArgumentError("META DATA SEEMS TO BE MISSING");
      }
      if (!(first_line >> num_nodes)) {
        return absl::InvalidArgumentError("NUM_NODES SEEMS TO BE MISSING");
      }
      if (!(first_line >> num_edges)) {
        return absl::InvalidArgumentError("NUM_EDGES SEEMS TO BE MISSING");
      }
      while (std::getline(input, line)) {
        NT u = 0;
        NT v = 0;
        int ins_del = 0;

        std::stringstream ss(line);
        ss >> ins_del;
        ss >> u;
        ss >> v;

        if (ins_del == 1) {
          graph.push_back({{u, v}, Mode::INSERTION});
        } else {
          graph.push_back({{u, v}, Mode::DELETION});
        }
      }
      return std::make_unique<DynamicGraphMemory<NT>>(graph, num_nodes);
    }
    return absl::UnimplementedError("The format " + hypergraph.format() + " is not implemented");
  }
};
using CCHHQRSMemory = DynDeltaInMemory<CCHHQRSEdgeOrientation>;
template <class Dyn>
using PackedCCHHQRSEdgeOrientationVector = PackedCCHHQRSEdgeOrientation<Dyn, InBucketsVector>;
template <class Dyn>
using PackedCCHHQRSEdgeOrientationList = PackedCCHHQRSEdgeOrientation<Dyn, InBucketsList>;
template <class Dyn>
using PackedCCHHQRSEdgeOrientationMap = PackedCCHHQRSEdgeOrientation<Dyn, InBucketsMap>;
using PackedCCHHQRSMemory = DynDeltaInMemory<PackedCCHHQRSEdgeOrientationVector>;
using PackedCCHHQRSMemoryList = DynDeltaInMemory<PackedCCHHQRSEdgeOrientationList>;
using PackedCCHHQRSMemoryMap = DynDeltaInMemory<PackedCCHHQRSEdgeOrientationMap>;

using LimitedBFSMemory = DynDeltaInMemory<LimitedBFSEdgeOrientation>;
using StrongEdgeOrientationMemory = DynDeltaInMemory<StrongEdgeOrientation>;
using ImprovedEdgeOrientationMemory = DynDeltaInMemory<ImprovedEdgeOrientation>;

REGISTER_IMPL_NAMED(CCHHQRSMemory, "cchhqrs");
REGISTER_IMPL_NAMED(LimitedBFSMemory, "limited_bfs");
REGISTER_IMPL_NAMED(StrongEdgeOrientationMemory, "strong_bfs");
REGISTER_IMPL_NAMED(ImprovedEdgeOrientationMemory, "improved_bfs");
REGISTER_IMPL_NAMED(PackedCCHHQRSMemory, "packed_cchhqrs");
REGISTER_IMPL_NAMED(PackedCCHHQRSMemoryList, "packed_cchhqrs_list");
REGISTER_IMPL_NAMED(PackedCCHHQRSMemoryMap, "packed_cchhqrs_map");

}  // namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation