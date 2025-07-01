
#include <google/protobuf/text_format.h>

#include <fstream>
#include <sstream>

#include "absl/strings/match.h"
#include "absl/strings/str_replace.h"
#include "absl/strings/substitute.h"
#include "app/app_io.pb.h"

namespace {
using dyn_delta_approx::app::app_io::ExperimentConfig;
using dyn_delta_approx::app::app_io::ExperimentResultMain;
using dyn_delta_approx::app::app_io::ExperimentResultPart;
}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; i++) {
    std::ifstream exp_file(argv[i]);
    if (!exp_file.good()) {
      std::cerr << "The file '" << argv[i] << "' must exist." << std::endl;
      return 1;
    }
    ExperimentResultPart part;
    if (!part.ParseFromIstream(&exp_file)) {
      std::cout << "something went wrong " << argv[i] << std::endl;
      exit(0);
    }
    std::ofstream exp_fileO(absl::StrReplaceAll(argv[i], {{"binary_proto", "textproto"}}));
    std::string result;
    google::protobuf::TextFormat::PrintToString(part, &result);
    exp_fileO << result;
  }
}