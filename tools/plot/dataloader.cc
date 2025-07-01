#include "dataloader.h"

#include <filesystem>
#include <fstream>

namespace dyn_delta_approx {
namespace tools {
namespace plot {
namespace {
using dyn_delta_approx::app::app_io::Visualisation;
}  // namespace

bool ExperimentResult::loadMain(std::ifstream& f) {
  if (!f.good()) {
    return false;
  }
  return main.ParseFromIstream(&f);
}
bool ExperimentResult::loadResultPart(std::ifstream& f) {
  ExperimentResultPart v;
  if (!v.ParseFromIstream(&f)) {
    return false;
  }
  vec.push_back(v);
  return true;
}
absl::Status ExperimentResult::finishLoading(
    bool prefix_name, const std::map<std::string, std::string>& rename_labels, bool has_main,
    bool print_params) {
  if (finished) {
    return absl::UnknownError("already finished");
  }
  finished = true;
  std::string prefix = path.filename();
  size_t added = 0;
  for (auto& v : vec) {
    for (auto r : v.results()) {
      if (prefix_name) {
        *r.mutable_run_config()->mutable_short_name() = prefix + "/" + r.run_config().short_name();
      }
      if (print_params) {
        std::stringstream conf;
        conf << r.run_config().short_name();
        for (auto& confs : r.run_config().algorithm_configs()) {
          for (auto& [k, v] : confs.bool_params()) {
            conf << "(" << k << ":" << v << ")";
          }
          for (auto& [k, v] : confs.double_params()) {
            conf << "(" << k << ":" << v << ")";
          }
          for (auto& [k, v] : confs.int64_params()) {
            conf << "(" << k << ":" << v << ")";
          }
          for (auto& [k, v] : confs.string_params()) {
            conf << "(" << k << ":" << v << ")";
          }
        }
        r.mutable_run_config()->set_short_name(conf.str());
      }
      if (rename_labels.find(r.run_config().short_name()) != rename_labels.end()) {
        *r.mutable_run_config()->mutable_short_name() =
            rename_labels.at(r.run_config().short_name());
      }
      if (replace.find(r.hypergraph().name()) != replace.end()) {
        *r.mutable_hypergraph() = replace[r.hypergraph().name()];
      }
      res.push_back(r);
      added++;
    }
  }
  if (has_main) {
    failed_ = std::vector<FailedExperiment>(main.failed_experiments().begin(),
                                            main.failed_experiments().end());
  }
  if (has_main && false &&
      main.id_of_finished_exp_size() - main.failed_experiments_size() != added) {
    std::cerr << "[WARN]" << prefix << " " << main.id_of_finished_exp_size() << " vs. " << added
              << std::endl;
    return absl::AbortedError("Not all experiments loaded.");
  }
  vec.clear();
  return absl::OkStatus();
}
absl::StatusOr<std::vector<ExperimentResult>> load_data(
    std::string root_path, decltype(Visualisation().experiment_paths()) dirs,
    std::map<std::string, std::string> rename_labels, const std::string& correct_path,
    bool prefix_name, bool print_params) {
  std::vector<ExperimentResult> results;
  for (auto& dir : dirs) {
    std::filesystem::path path(root_path + "/" + dir);
    std::filesystem::directory_iterator iter(path);
    ExperimentResult res(path, correct_path != "", root_path + "/" + correct_path);
    bool has_main = false;
    for (const auto& entry : iter) {
      if (entry.is_regular_file() &&
          entry.path().extension() == std::filesystem::path(".binary_proto")) {
        auto stem = entry.path().stem().string();
        std::ifstream fiestream(entry.path().string());
        if (stem == "result-main") {
          if (!res.loadMain(fiestream)) {
            return absl::UnknownError("result-main did not load.");
          }
          has_main = true;
        } else {
          if (!res.loadResultPart(fiestream)) {
            return absl::UnknownError(entry.path().string() + " did not load.");
          }
        }
      }
    }
    auto res1 = res.finishLoading(prefix_name, rename_labels, has_main, print_params);
    if (!res1.ok()) {
      return res1;
    }
    results.push_back(res);
  }
  std::cout << "Loaded " << results.size() << " dirs" << std::endl;
  return results;
}
}  // namespace plot
}  // namespace tools
}  // namespace dyn_delta_approx