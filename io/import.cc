#include <fstream>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "app/app_io.pb.h"
#include "ds/modifiable_hypergraph.h"
#include "ds/undirected_graph.h"
#include "google/protobuf/text_format.h"
#include "io/hgr_reader.h"
#include "io/hgr_writer.h"
#include "io/mtx_reader.h"
#include "io/seq_reader.h"
namespace {
using dyn_delta_approx::app::app_io::Hypergraph;
using dyn_delta_approx::app::app_io::HypergraphCollection;
using dyn_delta_approx::ds::StandardIntegerHypergraph;
}  // namespace

ABSL_FLAG(std::string, collection_name, "<noname>", "The name of the collection to write.");
ABSL_FLAG(std::string, version_collection, "1.0", "The version of the collection");
ABSL_FLAG(std::string, description, "", "The description of the collection to write.");
ABSL_FLAG(int, mod, 100, "Modulo which number randomness is used.");
ABSL_FLAG(bool, generate_random, true, "If true generates random<mod> entries");
ABSL_FLAG(bool, validating, true, "If true generates random<mod> entries");

ABSL_FLAG(std::string, sort, "", "Which sort to be saved.");
ABSL_FLAG(std::string, input_edges_weight, "unweighted",
          "Whether input edges are weighted or not.");
ABSL_FLAG(std::string, input_nodes_weight, "unweighted",
          "Whether input nodes are weighted or not.");
ABSL_FLAG(std::string, input_format, "mtx", "The format of the files.");
int main(int argc, char** argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  absl::ParseCommandLine(argc, argv);
  if (argc < 2) {
    std::cerr << "Please provide at least 2 params" << std::endl;
    return 1;
  }

  HypergraphCollection collection;
  std::vector<Hypergraph> hypergraphs;
  int mod = absl::GetFlag(FLAGS_mod);
  std::string collection_name = absl::GetFlag(FLAGS_collection_name);
  std::string sort = absl::GetFlag(FLAGS_sort);
  bool generate_random = absl::GetFlag(FLAGS_generate_random);
  std::string input_format = absl::GetFlag(FLAGS_input_format);
  bool validating = absl::GetFlag(FLAGS_validating);
  bool entries = false;
  for (int i = 2; i < argc; i++) {
    std::string name(argv[i]);
    if (name == "--") {
      entries = true;
      continue;
    }
    if (!entries) {
      continue;
    }
    std::cerr << name << std::endl;
    std::unique_ptr<dyn_delta_approx::ds::UndirectedGraph<size_t>> hypergraph_U;
    std::ifstream istream(name);
    if (input_format == "seq") {
      auto hypergraph_s =
          dyn_delta_approx::io::readSeq<dyn_delta_approx::ds::UndirectedGraph<size_t>>(istream);
      if (!hypergraph_s.ok()) {
        std::cout << "Skipping " << name << std::endl;
        continue;
      }

      hypergraph_U = std::move(hypergraph_s.value());
    }
    if (input_format == "mtx") {
      auto hypergraph_s =
          dyn_delta_approx::io::readMtxGraph<dyn_delta_approx::ds::UndirectedGraph<size_t>>(
              istream);
      if (!hypergraph_s.ok()) {
        std::cout << "Skipping " << name << std::endl;
        continue;
      }

      hypergraph_U = std::move(hypergraph_s.value());
    }
    auto& hypergraph = *hypergraph_U.get();
    if (validating && !hypergraph.validate()) {
      std::cout << "Validate failed " << name << std::endl;
      continue;
    }
    Hypergraph h_config;
    h_config.set_name(name);
    h_config.set_format(input_format);
    h_config.set_file_path(name);
    h_config.set_node_count(hypergraph.currentNumNodes());
    h_config.set_edge_count(hypergraph.currentNumEdges());
    h_config.set_collection(collection_name);
    h_config.set_sort(sort);
    h_config.set_node_weight_type(absl::GetFlag(FLAGS_input_nodes_weight));
    h_config.set_edge_weight_type(absl::GetFlag(FLAGS_input_edges_weight));

    hypergraphs.push_back(h_config);
  }
  collection.set_collection_name(collection_name);
  *collection.mutable_version() = absl::GetFlag(FLAGS_version_collection);
  *collection.mutable_hypergraphs() = {hypergraphs.begin(), hypergraphs.end()};
  std::ofstream exp_file("collection.textproto");
  std::string result;
  google::protobuf::TextFormat::PrintToString(collection, &result);
  exp_file << result;
}