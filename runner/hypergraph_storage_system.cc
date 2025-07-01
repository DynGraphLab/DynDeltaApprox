#include "runner/hypergraph_storage_system.h"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "google/protobuf/text_format.h"
#include "hypergraph_storage_system.h"

namespace dyn_delta_approx::runner::hypergraph_storage_system {
namespace {
using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;
using dyn_delta_approx::app::app_io::HypergraphStorageSystemConfig;
}  // namespace
HypergraphStorageSystem::HypergraphStorageSystem(absl::string_view directory)
    : _directory(directory) {
  reload();
}
HypergraphStorageSystemConfig HypergraphStorageSystem::config() const { return _config; }

std::vector<std::string> HypergraphStorageSystem::collection_names() const {
  std::vector<std::string> _collection_names;
  for (const auto& c : _hypergraph_collections) {
    _collection_names.push_back(c.collection_name());
  }
  return _collection_names;
}

size_t HypergraphStorageSystem::hypergraph_count() const {
  size_t count = 0;
  for (const auto& c : _hypergraph_collections) {
    count += c.hypergraphs().size();
  }
  return count;
}

std::vector<HypergraphCollection> HypergraphStorageSystem::collections() const {
  return _hypergraph_collections;
}
void HypergraphStorageSystem::reload() {
  _hypergraph_collections.clear();
  // look up  magic file
  std::ifstream collection_file(_directory + "/storage.textproto");
  if (!collection_file.good()) {
    std::cerr << "storage.textproto not found" << std::endl;
    exit(1);
  }
  std::stringstream buffer;
  buffer << collection_file.rdbuf();

  if (!google::protobuf::TextFormat::ParseFromString(buffer.str(), &_config)) {
    std::cerr << "Parsing error in '" << _directory + "/storage.textproto" << "'" << std::endl;
    exit(1);
  }
  for (const auto& dirEntry : recursive_directory_iterator(_directory)) {
    if (dirEntry.is_directory()) {
      find_collection(dirEntry.path());
    }
  }
}

std::string HypergraphStorageSystem::root_path() const { return _directory; }

void HypergraphStorageSystem::find_collection(const std::filesystem::path& path) {
  //  Look for collection.textproto
  std::ifstream collection_file(path.string() + "/collection.textproto");
  if (collection_file.good()) {
    // Read
    std::stringstream buffer;
    buffer << collection_file.rdbuf();
    HypergraphCollection collection;
    if (!google::protobuf::TextFormat::ParseFromString(buffer.str(), &collection)) {
      std::cerr << "Parsing error in '" << path.string() + "/collection.textproto" << "'"
                << std::endl;
      exit(1);
    }

    *collection.mutable_root_path() = path.string();
    // Update file paths
    for (auto& hypergraph : *collection.mutable_hypergraphs()) {
      *hypergraph.mutable_file_path() = collection.root_path() + "/" + hypergraph.file_path();
    }
    _hypergraph_collections.push_back(collection);
  }
}
}  // namespace dyn_delta_approx::runner::hypergraph_storage_system
