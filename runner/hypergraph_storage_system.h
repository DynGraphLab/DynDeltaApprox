#pragma once
#include <filesystem>

#include "absl/strings/string_view.h"
#include "app/app_io.pb.h"

namespace dyn_delta_approx::runner::hypergraph_storage_system {
namespace {
using dyn_delta_approx::app::app_io::Hypergraph;
using dyn_delta_approx::app::app_io::HypergraphCollection;
using dyn_delta_approx::app::app_io::HypergraphStorageSystemConfig;

}  // namespace
class HypergraphStorageSystem {
 public:
  HypergraphStorageSystem(absl::string_view directory);
  std::vector<std::string> collection_names() const;
  std::vector<HypergraphCollection> collections() const;
  void reload();
  HypergraphStorageSystemConfig config() const;
  size_t hypergraph_count() const;
  std::string root_path() const;

 private:
  HypergraphStorageSystemConfig _config;
  std::string _directory;
  std::vector<HypergraphCollection> _hypergraph_collections;
  void find_collection(const std::filesystem::path& path);
};
}  // namespace dyn_delta_approx::runner::hypergraph_storage_system