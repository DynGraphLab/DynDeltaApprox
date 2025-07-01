#include <google/protobuf/text_format.h>

#include <fstream>

#include "absl/algorithm/container.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "absl/strings/match.h"
#include "absl/strings/str_replace.h"
#include "absl/strings/substitute.h"
#include "app/app_io.pb.h"
#include "runner/hypergraph_storage_system.h"

namespace {
using dyn_delta_approx::app::app_io::ExperimentConfig;
using dyn_delta_approx::app::app_io::Hypergraph;
using dyn_delta_approx::app::app_io::HypergraphFilter;
using dyn_delta_approx::runner::hypergraph_storage_system::HypergraphStorageSystem;
}  // namespace

ABSL_FLAG(std::string, experiment_path, "", "The path to the folder including definitions file.");
ABSL_FLAG(std::string, hypergraph_filter, "",
          "HypergraphFilter textproto. Note that if sort,*_weight_types, or "
          "format is empty "
          "all are accepted.");
ABSL_FLAG(std::string, data_path, "",
          "Path to the data file for hypergraphs following the structure");
ABSL_FLAG(std::string, run_configs, "", "Run config object as textproto.");
ABSL_FLAG(std::string, experiment_name, "", "Experiment name");
ABSL_FLAG(bool, exclusive, false, "Is this process exclusive");
ABSL_FLAG(int, concurrent_processes, 1, "How many task are run at the same time.");
ABSL_FLAG(int, repetitions, 1, "How many repetitions of this experiment.");
ABSL_FLAG(int, sample, -1, "How many instances should be used.");
ABSL_FLAG(std::string, sample_by, "m/n",
          "Which sample method should be used uniform sampling based on m/n");

int main(int argc, char** argv) {
  absl::SetProgramUsageMessage("Builds a experiment.textproto");
  absl::ParseCommandLine(argc, argv);

  std::string exp_path = absl::GetFlag(FLAGS_experiment_path);
  std::string exp_name = absl::GetFlag(FLAGS_experiment_name);
  std::string run_configs = absl::GetFlag(FLAGS_run_configs);
  std::string data_path = absl::GetFlag(FLAGS_data_path);
  std::string hypergraph_filter_s = absl::GetFlag(FLAGS_hypergraph_filter);

  bool exclusive = absl::GetFlag(FLAGS_exclusive);
  ExperimentConfig experiment_config;
  ExperimentConfig experiment_config2;
  HypergraphFilter hypergraph_filter;
  if (!google::protobuf::TextFormat::ParseFromString(hypergraph_filter_s, &hypergraph_filter)) {
    std::cerr << "Error parsing 'hypergraph_filter'." << std::endl;
    return 1;
  }
  // max edges/nodes can be ommitted
  hypergraph_filter.set_max_edges(hypergraph_filter.max_edges() == 0
                                      ? std::numeric_limits<int64_t>::max()
                                      : hypergraph_filter.max_edges());
  // TODO leonie filter
  hypergraph_filter.set_max_nodes(hypergraph_filter.max_nodes() == 0
                                      ? std::numeric_limits<int64_t>::max()
                                      : hypergraph_filter.max_nodes());
  *experiment_config.mutable_filter() = hypergraph_filter;

  experiment_config.set_repetitions(absl::GetFlag(FLAGS_repetitions));

  if (!google::protobuf::TextFormat::ParseFromString(run_configs, &experiment_config2)) {
    std::cerr << "Error parsing 'run_configs'." << std::endl;
    return 1;
  }

  *experiment_config.mutable_experiment_name() = exp_name;
  *experiment_config.mutable_run_configs() = experiment_config2.run_configs();
  experiment_config.set_exclusive(exclusive);

  HypergraphStorageSystem hyper_storage_system(data_path);
  *experiment_config.mutable_root_path() = hyper_storage_system.root_path();
  for (const auto& name : hyper_storage_system.collection_names()) {
    std::cout << name << std::endl;
  }
  std::cout << "Found " << hyper_storage_system.collection_names().size()
            << " collections containing " << hyper_storage_system.hypergraph_count()
            << " hypergraphs. Processing." << std::endl;

  std::vector<Hypergraph> hypergraphs;
  for (auto& collection : hyper_storage_system.collections()) {
    if (absl::c_any_of(
            hypergraph_filter.hypergraph_collections(),
            [&collection](auto a) { return a.name() == collection.collection_name(); }) ||
        hypergraph_filter.hypergraph_collections().empty()) {
      for (auto& hypergraph : *collection.mutable_hypergraphs()) {
        // TODO leonie filter
        if (hypergraph.edge_count() <= hypergraph_filter.max_edges() &&
            hypergraph.edge_count() >= hypergraph_filter.min_edges() &&
            hypergraph.node_count() <= hypergraph_filter.max_nodes() &&
            hypergraph.node_count() >= hypergraph_filter.min_nodes() &&
            (absl::c_any_of(hypergraph_filter.node_weight_types(),
                            [&hypergraph](auto a) { return a == hypergraph.node_weight_type(); }) ||
             hypergraph_filter.node_weight_types().empty()) &&
            (absl::c_any_of(hypergraph_filter.edge_weight_types(),
                            [&hypergraph](auto a) { return a == hypergraph.edge_weight_type(); }) ||
             hypergraph_filter.edge_weight_types().empty()) &&
            (absl::c_any_of(hypergraph_filter.sorts(),
                            [&hypergraph](auto a) { return a == hypergraph.sort(); }) ||
             hypergraph_filter.sorts().empty()) &&
            (absl::c_any_of(hypergraph_filter.collections(),
                            [&hypergraph](auto a) { return a == hypergraph.collection(); }) ||
             hypergraph_filter.collections().empty()) &&
            (hypergraph_filter.format() == hypergraph.format() ||
             hypergraph_filter.format().empty())) {
          // adjust path to root directory
          std::string f_path(absl::StrReplaceAll(hypergraph.file_path(),
                                                 {{hyper_storage_system.root_path(), ""}}));
          *(hypergraph.mutable_file_path()) = f_path;
          hypergraphs.push_back(hypergraph);
        }
      }
    }
  }
  if (absl::GetFlag(FLAGS_sample) > 0) {
    std::function<double(Hypergraph)> sample_function = [](auto H) {
      return ((double)H.edge_count()) / ((double)H.node_count());
    };
    std::sort(hypergraphs.begin(), hypergraphs.end(),
              [&](auto a, auto b) { return sample_function(a) < sample_function(b); });
    std::vector<Hypergraph> sampled;
    int offset = hypergraphs.size() / absl::GetFlag(FLAGS_sample);
    for (int i = 0; i < hypergraphs.size(); i += offset) {
      sampled.push_back(hypergraphs[i]);
    }
    hypergraphs = sampled;
  }
  std::cout << "Found " << hypergraphs.size() << " hypergraphs for this experiment." << std::endl;
  std::sort(hypergraphs.begin(), hypergraphs.end(),
            [](auto a, auto b) { return a.edge_count() < b.edge_count(); });
  *experiment_config.mutable_hypergraphs() = {hypergraphs.begin(), hypergraphs.end()};
  experiment_config.set_concurrent_processes(absl::GetFlag(FLAGS_concurrent_processes));
  std::ofstream exp_file(exp_path + "/experiment.textproto");
  std::string result;
  google::protobuf::TextFormat::PrintToString(experiment_config, &result);
  exp_file << result;
}