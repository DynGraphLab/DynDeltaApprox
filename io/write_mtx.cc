#include <fstream>

#include "ds/modifiable_hypergraph.h"
#include "io/hgr_reader.h"
#include "io/hgr_writer.h"

namespace {
using dyn_delta_approx::ds::StandardIntegerHypergraph;
}
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: ./write_mtx <hgr> " << std::endl;
    return 1;
  }
  for (int i = 1; i < argc; i++) {
    std::ifstream istream(argv[i]);
    std::ofstream ostream2(std::string(argv[i]) + ".mtx");

    auto hypergraph = dyn_delta_approx::io::readHypergraph<StandardIntegerHypergraph>(istream);
    StandardIntegerHypergraph hg = *hypergraph->get();
    dyn_delta_approx::io::writeHypergraphMTXUnigraph(hg, ostream2);
  }
}