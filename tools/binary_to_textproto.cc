#include <fstream>

#include "app/app_io.pb.h"
#include "google/protobuf/text_format.h"
#include "runner/hypergraph_storage_system.h"

namespace {
using ExperimentResultPart = dyn_delta_approx::app::app_io::ExperimentResultPart;
using Hypergraph = dyn_delta_approx::app::app_io::Hypergraph;

using HypergraphStorageSystem =
    dyn_delta_approx::runner::hypergraph_storage_system::HypergraphStorageSystem;
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << " Please supply more data. Usage ./update_hypergraph_info "
                 " <binary proto file one> ..."
              << std::endl;
    return 1;
  }

  for (int i = 1; i < argc; i++) {
    ExperimentResultPart part;
    {
      std::ifstream istream(argv[i]);
      part.ParseFromIstream(&istream);
    }
    std::ofstream ostream(std::string(argv[i]) + ".text");
    std::string textproto_string;
    google::protobuf::TextFormat::PrintToString(part, &textproto_string);
    ostream << textproto_string;
    std::cout << textproto_string << std::endl;
  }
}