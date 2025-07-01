#include <fstream>

#include "ds/undirected_graph.h"
#include "io/binary_writer.h"
#include "io/graph_reader.h"
namespace {
using Graph = dyn_delta_approx::ds::UndirectedGraph<size_t>;
}

int main(int argc, char** argv) {
  for (int i = 1; i < argc; i++) {
    std::ifstream input(argv[i]);
    std::ofstream output(std::string(argv[i]) + ".bin");
    auto graph_s = dyn_delta_approx::io::readGraph<Graph>(input, true);
    std::cout << "Finished loading" << std::endl;
    if (!graph_s.ok()) {
      std::cout << graph_s.status() << std::endl;
      exit(1);
    }
    dyn_delta_approx::io::writeBinaryUndirectedGraph(*graph_s.value().get(), output, true);
  }
}