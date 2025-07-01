#include "plot.h"

#include <algorithm>
#include <any>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/str_replace.h"
#include "absl/strings/string_view.h"
#include "absl/strings/strip.h"
#include "app/app_io.pb.h"
#include "dataloader.h"
#include "gaussian_error.h"
#include "google/protobuf/map.h"
#include "google/protobuf/text_format.h"
#include "matplotlibcpp17/axes.h"
#include "matplotlibcpp17/figure.h"
#include "matplotlibcpp17/pyplot.h"
#include "pybind11/cast.h"
#include "pybind11/embed.h"
#include "pybind11/eval.h"
#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"
#include "pybind11/pytypes.h"
#include "src/google/protobuf/util/time_util.h"
#include "tools/plot/pivot_table.h"

namespace dyn_delta_approx {
namespace tools {
namespace plot {
namespace {
constexpr size_t numColors = 10;
constexpr size_t numLineStyles = 4;
constexpr size_t numMarkers = 10;

void save_textproto(const std::string& filename, app::app_io::Visualisation vis) {
  app::app_io::VisualisationsFile file;
  *file.add_visualisations() = vis;
  std::ofstream file_output(filename);
  std::string result;
  google::protobuf::TextFormat::PrintToString(file, &result);
  file_output << result;
}
constexpr absl::string_view kMinimizationProblem = "minimization_problem";
std::string prepare_title(const Visualisation& vis, const int capacity, const size_t count) {
  std::string cap = "rnd";
  if (capacity != -1) {
    cap = std::to_string(capacity);
  }
  return absl::StrReplaceAll(vis.title(),
                             {{"$num", std::to_string(count)},
                              {"$capacity", cap},
                              {"$performance_characteristic", vis.performance_characteristic()}});
}
std::map<std::string, std::tuple<std::string, int, int>> result;
std::vector<bool> colors_used(numColors* numLineStyles, false);

std::map<std::string, std::tuple<std::string, int, int>> color_map(
    const std::set<std::string>& names) {
  if (names.size() > numColors * numLineStyles) {
    std::cerr << "Too many in map" << std::endl;
    exit(1);
  }
  std::hash<std::string> hasher;
  for (auto c : names) {
    if (result.contains(c)) {
      continue;
    }
    if (result.contains(std::string(absl::StripAsciiWhitespace(c)))) {
      result[c] = result[std::string(absl::StripAsciiWhitespace(c))];
      continue;
    }
    std::string trimmed(absl::StripAsciiWhitespace(c));
    int color = hasher(c) % (numColors);
    while (colors_used[color]) {
      color += 4;
      color %= (numColors * numLineStyles);
    }
    colors_used[color] = true;
    auto indx = color % numColors;
    auto lIndx = color / numColors;
    result[trimmed] =
        std::make_tuple(std::string("C") + std::to_string(indx), lIndx, color % numMarkers);
    result[c] = std::make_tuple(std::string("C") + std::to_string(indx), lIndx, color % numMarkers);
  }
  return result;
}
std::vector<Result> filter_exactness(std::vector<Result> data, Visualisation vis) {
  auto bool_params = vis.bool_params();
  auto string_params = vis.string_params();
  bool exact_filter = bool_params["exact_filter"];
  bool exact_filter_step1 = bool_params["exact_filter_step1"];
  bool exact_filter_step_last1 = bool_params["exact_filter_step-1"];

  bool exact_only = bool_params["exact_only"];
  bool exact_only_step1 = bool_params["exact_only_step1"];
  bool allow_list_instance = bool_params["allow_list_instances"];
  if (allow_list_instance) {
    std::set<std::string> filter_set;
    for (int i = 0; i < 10; i++) {
      if (auto pram = string_params[std::string("allow_instance") + std::to_string(i)];
          pram != "") {
        filter_set.insert(pram);
      }
    }
    FilterBy<Result, std::string> filter(data, [](auto a) { return a.hypergraph().name(); });
    data = filter.having([&](auto a) { return filter_set.contains(a); });
    std::cout << "[INFO] Filtered allow list of size" << filter_set.size() << "." << std::endl;
  }

  if (exact_filter) {
    std::cout << "[INFO] Filtering only at least one exact results." << std::endl;
    FilterBy<Result, std::string> filter(data, [](auto a) { return a.hypergraph().name(); });
    data = filter.at_least_one([](Result r) { return r.is_exact(); });
  }
  if (exact_filter_step1) {
    std::cout << "[INFO] Filtering only at least one exact in step 1 results." << std::endl;
    FilterBy<Result, std::string> filter(data, [](auto a) { return a.hypergraph().name(); });
    data = filter.at_least_one([](Result r) {
      return r.algorithm_run_informations_size() > 1 &&
             r.algorithm_run_informations()[1].is_exact();
    });
  }
  if (exact_filter_step_last1) {
    std::cout << "[INFO] Filtering only at least one exact in  last step 1 results." << std::endl;
    FilterBy<Result, std::string> filter(data, [](auto a) { return a.hypergraph().name(); });
    data = filter.at_least_one([](Result r) {
      return r.algorithm_run_informations_size() > 0 &&
             r.algorithm_run_informations()[r.algorithm_run_informations_size() - 1].is_exact();
    });
  }
  if (exact_only) {
    std::cout << "[INFO] Filtering only fully exact results." << std::endl;
    FilterBy<Result, std::string> filter(data, [](auto a) { return a.hypergraph().name(); });
    data = filter.all([](Result r) { return r.is_exact(); });
  }
  if (exact_only_step1) {
    std::cout << "[INFO] Filtering only fully exact in step 1 results." << std::endl;
    FilterBy<Result, std::string> filter(data, [](auto a) { return a.hypergraph().name(); });
    data = filter.all([](Result r) {
      return r.algorithm_run_informations_size() > 1 &&
             r.algorithm_run_informations()[1].is_exact();
    });
  }
  return data;
}
using RetTyp1 =
    std::tuple<std::map<std::string, std::map<std::string, GaussianError<double>>>, int>;
// @returns Map of per algorithm per instance.
absl::StatusOr<RetTyp1> normalize_by_optimum(Visualisation vis, std::vector<Result>& data,
                                             std::string sort = "all", int cap = 0) {
  std::cout << "[INFO] Begin: Plotting " << data.size() << " results." << std::endl;

  std::function<double(Result)> performance_char = [](Result r) -> double { return r.weight(); };
  if (vis.performance_characteristic() == "size") {
    performance_char = [](Result r) -> double { return r.size(); };
  }
  if (vis.performance_characteristic() == "quality") {
    performance_char = [](Result r) -> double { return r.quality(); };
  }
  if (vis.performance_characteristic() == "memory") {
    performance_char = [](Result r) -> double {
      return r.run_information().max_allocated_memory_in_mb();
    };
  }
  if (vis.performance_characteristic() == "last_edge_count") {
    performance_char = [](Result r) -> double {
      return r.algorithm_run_informations()[r.algorithm_run_informations_size() - 1].edge_count();
    };
  }
  if (vis.performance_characteristic() == "last_node_count") {
    performance_char = [](Result r) -> double {
      return r.algorithm_run_informations()[r.algorithm_run_informations_size() - 1].node_count();
    };
  }
  if (vis.performance_characteristic() == "last_runtime") {
    performance_char = [](Result r) -> double {
      return ::google::protobuf::util::TimeUtil::DurationToNanoseconds(
          r.algorithm_run_informations()[r.algorithm_run_informations_size() - 1].algo_duration());
    };
  }
  if (vis.performance_characteristic() == "runInformation_algoDuration" ||
      vis.performance_characteristic() == "runtime") {
    performance_char = [](Result r) -> double {
      return ::google::protobuf::util::TimeUtil::DurationToNanoseconds(
          r.run_information().algo_duration());
    };
  }
  auto bool_params = vis.bool_params();
  auto double_params = vis.double_params();
  auto string_params = vis.string_params();
  std::string normalize_by = string_params["normalize"];

  data = filter_exactness(data, vis);
  std::cout << "[INFO] Filtered: Plotting " << data.size() << " results." << std::endl;
  using GE = GaussianError<double>;
  std::function<GE(GE, GE)> optimal = [](auto a, auto b) { return std::max(a, b); };
  std::function<GE(GE, GE)> not_optimal = [](auto a, auto b) { return std::min(a, b); };
  if (bool_params[kMinimizationProblem]) {
    optimal = [](GE a, GE b) { return std::min(a, b); };
    not_optimal = [](GE a, GE b) { return std::max(a, b); };
  }
  // TODO maybe move to common preprocess step
  std::map<std::string, std::vector<Result>> by_heuristic;
  for (auto& r : data) {
    by_heuristic[r.run_config().short_name()].push_back(r);
  }
  // make sure all heuristics are included, this step does not guarantee, check after compression
  // first heuristic, second index instance
  std::vector<std::pair<std::string, std::map<std::string, GaussianError<double>>>> double_grouped;
  // TODO add specific values
  for (auto& [heuristic, data_points] : by_heuristic) {
    // group by instance
    GroupBy<Result, std::string> grouped(data_points,
                                         [](Result r) { return r.hypergraph().name(); });

    double_grouped.push_back(std::make_pair(heuristic, grouped.avg(performance_char)));
  }
  std::map<std::string, std::map<std::string, GaussianError<double>>> ratios;
  int idx = 0;
  int i = 0;
  if (double_grouped.size() == 0) {
    return absl::DataLossError("no data left.");
  }
  for (auto& [heur, points] : double_grouped) {
    if (double_grouped[idx].second.size() < points.size()) {
      idx = i;
    }
    i++;
  }
  auto& [heur, points] = double_grouped[idx];
  size_t count = 0;

  for (auto& [instance, val] : points) {
    // find min
    // TODO Add maximization
    GaussianError<double> opt_val = val;
    GaussianError<double> non_opt_val = val;

    bool not_found = false;
    for (auto& [alg, data2] : double_grouped) {
      if (data2.find(instance) == data2.end()) {
        std::cerr << "[WARN] heuristic " << alg << " does not contain " << instance << std::endl;
        not_found = true;
        break;
      }
      if (data2[instance] < 0.0) {
        std::cout << "[ERROR] negative value of " << data2[instance] << " at instance " << instance
                  << std::endl;
      }

      if (normalize_by != "") {
        if (alg == normalize_by) {
          opt_val = data2[instance];
        }
      } else {
        opt_val = optimal(data2[instance], opt_val);
        non_opt_val = not_optimal(data2[instance], non_opt_val);
      }
    }
    if (not_found) {
      continue;
    }
    if (opt_val == 0.0) {
      std::cout << "[WARN] ZERO opt: " << opt_val << " in instance: '" << instance
                << "'. will ignore this datapoint." << std::endl;
      continue;
    }
    if (non_opt_val < double_params["min_max_threshold"] * 1e9) {
      if (bool_params["verbose"]) {
        std::cout << "[VERBOSE] non_opt_val " << instance << " " << non_opt_val << " < "
                  << double_params["min_max_threshold"] << std::endl;
      }
      continue;
    }
    count++;
    if (bool_params["verbose"]) {
      std::cout << "[VERBOSE] opt_val " << instance << " " << opt_val << std::endl;
    }

    for (auto& [alg, data2] : double_grouped) {
      if (bool_params["verbose"]) {
        std::cout << "[INFO] " << alg << " " << instance << " " << data2[instance] / opt_val
                  << std::endl;
      }
      if (bool_params["raw_value"]) {
        ratios[alg][instance] = data2[instance];
      } else {
        ratios[alg][instance] = data2[instance] / opt_val;
      }
    }
  }
  return std::make_tuple(ratios, count);
}
}  // namespace

VisualisationTool::VisualisationTool(const std::string& vis_file) {
  std::ifstream f(vis_file);
  if (!f) {
    std::cerr << vis_file << " was not openable." << std::endl;
    exit(1);
  }
  std::stringstream buffer;
  buffer << f.rdbuf();
  if (!google::protobuf::TextFormat::ParseFromString(buffer.str(), &file)) {
    std::cerr << vis_file << " was not parsable." << std::endl;
    exit(1);
  }
  mod = pybind11::module::import("matplotlib.pyplot");
  seaborn = pybind11::module::import("seaborn");
  pandas = pybind11::module::import("pandas");
  skl = pybind11::module::import("sklearn.linear_model");
  np = pybind11::module::import("numpy");
  mod.attr("style").attr("use")(Args("bmh"));  // bmh
  constexpr int BIGGER_SIZE = 12;
  constexpr int MEDIUM_SIZE = 10;
  constexpr int SMALL_SIZE = 8;

  mod.attr("rc")(*Args("font"), **Kwargs("size"_a = BIGGER_SIZE));
  mod.attr("rc")(*Args("figure"), **Kwargs("titlesize"_a = BIGGER_SIZE));

  mod.attr("rc")(*Args("legend"), **Kwargs("fontsize"_a = SMALL_SIZE));
  mod.attr("rc")(*Args("xtick"), **Kwargs("labelsize"_a = SMALL_SIZE));
  mod.attr("rc")(*Args("ytick"), **Kwargs("labelsize"_a = SMALL_SIZE));
  mod.attr("rc")(*Args("legend"), **Kwargs("title_fontsize"_a = SMALL_SIZE));

  mod.attr("rc")(*Args("axes"), **Kwargs("titlesize"_a = SMALL_SIZE, "labelsize"_a = SMALL_SIZE));
  auto bool_params = file.bool_params();
  if (bool_params["true_type"]) {
    mod.attr("rc")(*Args("pdf"), **Kwargs("fonttype"_a = 42));
    mod.attr("rc")(*Args("ps"), **Kwargs("fonttype"_a = 42));
  }
  plt = matplotlibcpp17::pyplot::PyPlot(mod);
  // plt.plot()
}

void VisualisationTool::ensurePathsExists(std::string root_path, Visualisation& vis) {
  std::filesystem::create_directories(root_path + "/vis/" + vis.folder_name());
  vis.set_folder_name(root_path + "/vis/" + vis.folder_name());
}
#define lambda_bind(function_name) \
  [&](auto a, auto b, auto c, auto d) { return this->function_name(a, b, c, d); }

Visualisation ensureDefaults(const Visualisation& vis,
                             const app::app_io::VisualisationsFile& file) {
  auto default_string_params = file.string_params();
  auto default_int64_params = file.int64_params();
  auto default_double_params = file.double_params();
  auto default_bool_params = file.bool_params();
  auto default_rename_labels = file.rename_labels();
  auto default_display_labels = file.display_labels();

  for (auto [k, v] : vis.string_params()) {
    default_string_params[k] = v;
  }
  for (auto [k, v] : vis.double_params()) {
    default_double_params[k] = v;
  }
  for (auto [k, v] : vis.int64_params()) {
    default_int64_params[k] = v;
  }
  for (auto [k, v] : vis.bool_params()) {
    default_bool_params[k] = v;
  }
  for (auto [k, v] : vis.rename_labels()) {
    default_rename_labels[k] = v;
  }
  for (auto [k, v] : vis.display_labels()) {
    default_display_labels[k] = v;
  }
  Visualisation result = vis;
  *result.mutable_bool_params() = default_bool_params;
  *result.mutable_string_params() = default_string_params;
  *result.mutable_double_params() = default_double_params;
  *result.mutable_int64_params() = default_int64_params;
  *result.mutable_rename_labels() = default_rename_labels;
  *result.mutable_display_labels() = default_display_labels;

  return result;
}

absl::Status VisualisationTool::plot(std::string root_path) {
  for (auto vis : file.visualisations()) {
    // ensure default parameters are inserted
    vis = ensureDefaults(vis, file);
    auto bool_params = vis.bool_params();
    auto string_params = vis.string_params();
    auto data_s = dyn_delta_approx::tools::plot::load_data(
        root_path, vis.experiment_paths(),
        std::map<std::string, std::string>(vis.rename_labels().begin(), vis.rename_labels().end()),
        string_params["correct_path"], bool_params["prefix_names"], bool_params["print_params"]);
    if (!data_s.ok()) {
      return data_s.status();
    }
    auto data = data_s.value();
    if (string_params["ignore_sort0"] != "") {
      for (auto& v : data) {
        v.filter({string_params["ignore_sort0"]});
      }
    }
    ensurePathsExists(root_path, vis);
    if (vis.type() == "performance_profile") {
      if (auto st = plot_internal(data, vis, lambda_bind(plot_performance_profile_internal));
          !st.ok()) {
        return st;
      }
    } else if (vis.type() == "scatter_plot") {
      if (auto st = plot_internal(data, vis, lambda_bind(scatter_plot_internal)); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "table") {
      if (auto st = table(data, vis); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "stats_table") {
      if (auto st = stats_table(data, vis); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "efficency") {
      if (auto st = efficency(data, vis); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "stats") {
      if (auto st = stats(data, vis); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "stats_debug") {
      if (auto st = stats_debug(data, vis); !st.ok()) {
        return st;
      }

    } else if (vis.type() == "categorial_bar_plot") {
      if (auto st = categorial_bar_plot(data, vis); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "scatter_debug") {
      if (auto st = scatter_debug_plot(data, vis); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "jointplot") {
      if (auto st = jointplot_plot(data, vis); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "violin_plot") {
      if (auto st = violin_plot(data, vis); !st.ok()) {
        return st;
      }
    } else if (vis.type() == "write_csv") {
      if (auto st = write_csv(data, vis); !st.ok()) {
        return st;
      }
    } else {
      return absl::UnimplementedError(" type '" + vis.type() + "' is not implemented (yet).");
    }
  }
  return absl::OkStatus();
}

absl::Status VisualisationTool::plot_performance_profile_internal(Visualisation vis,
                                                                  std::vector<Result>& data,
                                                                  std::string sort, int cap) {
  auto bool_params = vis.bool_params();
  auto double_params = vis.double_params();
  auto string_params = vis.string_params();
  auto int64_params = vis.int64_params();

  std::function<double(double, double)> optimal = [](double a, double b) { return std::max(a, b); };
  std::function<double(double, double)> not_optimal = [](double a, double b) {
    return std::min(a, b);
  };
  if (bool_params[kMinimizationProblem]) {
    optimal = [](double a, double b) { return std::min(a, b); };
    not_optimal = [](double a, double b) { return std::max(a, b); };
    // comperator stays the same comparator = [](double a, double b) { return a < b; };
  }
  auto ratios_s = normalize_by_optimum(vis, data, sort, cap);
  if (!ratios_s.ok()) {
    return ratios_s.status();
  }
  auto [ratios_m, count] = std::move(ratios_s.value());
  if (bool_params["verbose"]) {
    std::cout << "[VERBOSE] count " << count << std::endl;
  }
  // ratios stores the number of entries at x-axis point x
  std::map<std::string, std::map<double, size_t>> ratios;
  for (const auto& [algo, data] : ratios_m) {
    std::string min_instance = data.begin()->first;
    auto min_val = data.begin()->second;
    for (const auto& [instance, value] : data) {
      ratios[algo][value.getValue()] += 1;  // TODO pass on uncertainity
      if (value < min_val) {
        min_val = value;
        min_instance = instance;
      }
    }
    std::cout << algo << " Instance " << min_instance << " " << min_val << std::endl;
  }
  std::map<double, std::map<std::string, double>> displays;
  double factor = 1.0 / ((double)ratios_m.begin()->second.size());

  std::set<double> x_axis;
  x_axis.insert(1.0);
  for (auto& [alg, ratio] : ratios) {
    if (ratio.size() == 0) {
      std::cout << "[WARN] empty ratio. Something is wrong" << std::endl;
    }

    double i = 0;
    for (auto [r, c] : ratio) {
      x_axis.insert(r);
      i += c;
      displays[r][alg] = (i)*factor;
      if (bool_params["verbose"]) {
        std::cout << "[VERBOSE]" << alg << " " << r << " " << displays[r][alg] << std::endl;
      }
    }

    // TODO (add avg output)
    //  std::iota(y_values.begin(), y_values.end(), 0);
    //  for (auto y : y_values) {
    //  y /= (double)y_values.size();
    // }

    // TODO optimum
  }
  if (!bool_params[kMinimizationProblem]) {
    for (auto& [r, map] : displays) {
      for (auto& [_, f] : map) {
        f = 1.0 - f;
      }
    }
  }
  // map is ordered by <
  std::map<std::string, std::vector<double>> y_axis;

  std::vector<double> x_axis_vec;
  // x_axis_vec.push_back(0.0);
  for (auto x : x_axis) {
    x_axis_vec.push_back(x);
  }
  std::map<std::string, double> next_lower_value;
  if (!bool_params[kMinimizationProblem]) {
    for (auto& [algo, _] : ratios) {
      next_lower_value[algo] = 1.0;
    }
  }
  for (auto& [x_value, map] : displays) {
    for (auto [k, v] : ratios) {
      if (map.find(k) == map.end()) {
        map[k] = next_lower_value[k];
      }
      next_lower_value[k] = map[k];
    }
    for (auto [s, y_value] : map) {
      y_axis[s].push_back(y_value);
    }
  }
  auto least_optimal_value = x_axis_vec.front();
  std::cout << "Least: " << least_optimal_value << std::endl;
  for (const auto& x : x_axis_vec) {
    least_optimal_value = not_optimal(least_optimal_value, x);
  }
  std::cout << "Least: " << least_optimal_value << std::endl;
  if (!bool_params[kMinimizationProblem]) {
    std::reverse(x_axis_vec.begin(), x_axis_vec.end());
  }
  x_axis_vec.push_back(least_optimal_value * (bool_params[kMinimizationProblem] ? 1.1 : 0.9));
  for (auto& [k, v] : y_axis) {
    if (!bool_params[kMinimizationProblem]) {
      std::reverse(v.begin(), v.end());
    }
    v.push_back(1.0);
  }
  int n = 0;
  std::vector<std::string> styles = {"-", "--", "-.", ":"};
  std::vector<std::string> markers = {
      ".", "o", "v", "^", "<", ">", "8", "s", "p", "*",
  };

  auto fig =
      plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(double_params["fig_width_inch"],
                                                             double_params["fig_height_inch"])));
  auto ax = fig.add_axes(Args(std::vector<double>({0.125, 0.175, 0.80, 0.75})));
  std::vector<std::pair<std::string, std::vector<double>>> y_axis2;
  std::copy(y_axis.begin(), y_axis.end(), std::back_inserter(y_axis2));
  if (bool_params["sort_legend_avg"]) {
    std::map<std::string, double> val;
    for (auto& [name, values] : y_axis2) {
      val[name] = std::transform_reduce(
                      values.begin(), values.end(), x_axis_vec.begin(), 0.0,
                      [](auto a, auto b) { return a + b; }, [](auto a, auto b) { return a * b; }) /
                  ((double)x_axis_vec.size());
    }
    std::sort(y_axis2.begin(), y_axis2.end(),
              [&](auto& a, auto& b) { return val[a.first] > val[b.first]; });
  }
  std::set<std::string> names;
  for (auto& [k, v] : y_axis2) {
    names.insert(k);
  }
  auto cmap = color_map(names);
  for (auto& [algo, y_a] : y_axis2) {
    auto [color, line_style, mark] = cmap[algo];
    ax.plot(Args(x_axis_vec, y_a),
            Kwargs("label"_a = algo, "alpha"_a = 0.7,
                   "drawstyle"_a = (bool_params[kMinimizationProblem] ? "steps-post" : "steps-pre"),
                   "linestyle"_a = styles[line_style], "markersize"_a = 4, "markevery"_a = 0.1,
                   "marker"_a = markers[mark], "color"_a = color));
    n++;
  }
  if (double_params["x_limit"] != 0) {
    ax.set_xlim(Args(1.0, double_params["x_limit"]));
  } else {
    ax.set_xlim(Args(1.0, least_optimal_value * (bool_params[kMinimizationProblem] ? 1.1 : 0.9)));
  }
  if (bool_params["x_log_scale"]) {
    ax.set_xscale(Args("log"));
  }
  ax.set_xlabel(Args("$\\tau$"));
  ax.set_ylabel(Args("Fraction of Instances"));
  if (!bool_params["legend_off"]) {
    std::string legend_pos =
        string_params["legend_pos"] == "" ? "best" : string_params["legend_pos"];
    ax.legend(Args(), Kwargs("loc"_a = legend_pos));
  }
  if (bool_params["legend_seperate"]) {
    int rows_count = 1;
    if (int64_params["legend_rows"] != 0) {
      rows_count = int64_params["legend_rows"];
    }
    auto [fig_legend, ax_legend] = plt.subplots();
    auto legend = ax_legend.legend(
        ax.unwrap().attr("get_legend_handles_labels")(),
        Kwargs("ncol"_a = (int)std::ceil(((double)y_axis2.size()) / ((double)rows_count)),
               "loc"_a = "center"));  // ncol=2, loc='center'
    auto renderer = fig_legend.unwrap().attr("canvas").attr("get_renderer")();
    auto bbox =
        legend.unwrap()
            .attr("get_window_extent")(*Args(renderer))
            .attr("transformed")(fig_legend.unwrap().attr("dpi_scale_trans").attr("inverted")());
    ax_legend.unwrap().attr("axis")(*Args("off"));

    fig_legend.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_performance_profile_" +
                            std::to_string(cap) + "_" + sort + ".legend.pdf"),
                       Kwargs("bbox_inches"_a = bbox));
  }
  //   # Get the bounding box of the legend
  // renderer = fig_legend.canvas.get_renderer()
  // bbox = legend.get_window_extent(renderer).transformed(fig_legend.dpi_scale_trans.inverted())

  // # Save the legend with the adjusted bounding box
  // fig_legend.savefig('legend.pdf', bbox_inches=bbox)
  //     # Save the legend to a separate PDF
  // fig_legend, ax_legend = plt.subplots()
  // ax_legend.axis('off')  # Hide axes
  // ax_legend.legend(*ax.get_legend_handles_labels())  # Add legend to new figure
  // fig_legend.savefig('legend.pdf', bbox_inches='tight')

  // # Save the plot without the legend
  // ax.legend().remove()  # Remove legend from original plot
  // fig.savefig('plot.pdf')
  ax.set_title(Args(prepare_title(vis, cap, count)));
  fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_performance_profile_" +
                   std::to_string(cap) + "_" + sort + ".png"));
  fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_performance_profile_" +
                   std::to_string(cap) + "_" + sort + ".pdf"),
              Kwargs("bbox_inches"_a = "tight"));
  save_textproto(vis.folder_name() + "/" + vis.file_prefix() + "_performance_profile_" +
                     std::to_string(cap) + "_" + sort + ".textproto",
                 vis);
  std::cout << "Plotted "
            << (vis.folder_name() + "/" + vis.file_prefix() + "_performance_profile_" +
                std::to_string(cap) + "_" + sort + ".pdf")
            << std::endl
            << (vis.folder_name() + "/" + vis.file_prefix() + "_performance_profile_" +
                std::to_string(cap) + "_" + sort + ".png")
            << std::endl;
  plt.clf(Args(), Kwargs());
  // TODO add min/max settings
  return absl::OkStatus();
}

absl::Status VisualisationTool::plot_internal(
    std::vector<ExperimentResult>& data, Visualisation vis,
    std::function<absl::Status(Visualisation, std::vector<Result>&, std::string, int)> function) {
  auto transformed = transform_by_sort(data, vis);
  for (auto& [sort, data_points] : transformed) {
    std::cout << "Plotting " << sort << std::endl;
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = function(vis, filtered, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      auto plotted = function(vis, data_points, sort, 0);
      if (!plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::table_internal(Visualisation vis, std::vector<Result>& data2,
                                               std::vector<FailedExperiment>& failed,
                                               const std::set<std::string> index,
                                               const std::set<std::string> pivot, std::string sort,
                                               int64_t cap) {
  auto string_params = vis.string_params();
  auto bool_params = vis.bool_params();
  std::vector<Result> data;

  if (bool_params["special_copy"]) {
    std::set<std::string> in_data = {"min_degree", "max_degree", "connected_components", "b_edges",
                                     "a_nodes"};
    std::copy_if(data2.begin(), data2.end(), std::back_inserter(data),
                 [&](const auto& a) { return in_data.contains(a.run_config().short_name()); });
  } else {
    data = data2;
  }
  data = filter_exactness(data, vis);
  GroupBy<Result, std::array<std::string, 2>> groupby_instance_run_config(
      data, [](Result result) -> std::array<std::string, 2> {
        return {result.hypergraph().name(), result.run_config().short_name()};
      });
  auto grouped_inst = groupby_instance_run_config.agg<Result>(
      [](Result r) { return r; },
      [](Result a, Result b) {
        *b.mutable_run_information()->mutable_algo_duration() =
            ::google::protobuf::util::TimeUtil::NanosecondsToDuration(
                ::google::protobuf::util::TimeUtil::DurationToNanoseconds(
                    a.run_information().algo_duration()) +
                ::google::protobuf::util::TimeUtil::DurationToNanoseconds(
                    b.run_information().algo_duration()));
        return b;
      },
      [](Result a, size_t s) {
        *a.mutable_run_information()->mutable_algo_duration() =
            ::google::protobuf::util::TimeUtil::NanosecondsToDuration(
                ::google::protobuf::util::TimeUtil::DurationToNanoseconds(
                    a.run_information().algo_duration()) /
                s);
        return a;
      },
      Result{});
  std::vector<Result> as_v;
  std::transform(grouped_inst.begin(), grouped_inst.end(), std::back_inserter(as_v),
                 [](auto a) { return a.second; });
  GroupBy<Result, std::array<std::map<std::string, Any>, 2>> grouped(
      bool_params["group_per_instance"] ? as_v : data,
      [&](Result result) -> std::array<std::map<std::string, Any>, 2> {
        std::map<std::string, Any> res1, res2;
        for (auto i : index) {
          res1.insert(std::make_pair(i, performance_characteristics.at(i)(result)));
        }
        for (auto i : pivot) {
          res2.insert(std::make_pair(i, performance_characteristics.at(i)(result)));
        }
        return {res1, res2};
      });
  GroupBy<FailedExperiment, std::array<std::map<std::string, Any>, 2>> failed_group(
      failed, [&](FailedExperiment result) -> std::array<std::map<std::string, Any>, 2> {
        std::map<std::string, Any> res1, res2;
        for (auto i : index) {
          res1.insert(std::make_pair(i, performance_characteristics_failed.at(i)(result)));
        }
        for (auto i : pivot) {
          res2.insert(std::make_pair(i, performance_characteristics_failed.at(i)(result)));
        }
        return {res1, res2};
      });
  std::cout << "FAiled: " << failed_group.different_element_count() << std::endl;
  PivotTable pivot_table(
      string_params["mean"] == "max" ? grouped.max<double>([&](Result r) {
        return (performance_characteristics.at(vis.performance_characteristic())(r).to_double());
      })
      : string_params["mean"] == "geometric"
          ? grouped.geo_mean_log<double>([&](Result r) -> double {
              return (
                  performance_characteristics.at(vis.performance_characteristic())(r).to_double());
            })
          : grouped.agg<double>(
                [&](Result r) -> double {
                  return (performance_characteristics.at(vis.performance_characteristic())(r)
                              .to_double());
                },
                [](auto a, auto b) { return a + b; },
                [](auto a, size_t n) { return a / ((double)n); }),
      failed_group.agg<app::app_io::FailReason>(
          [](auto FR) { return FR.reason(); }, [](auto a, auto b) { return b; },
          [](auto a, size_t s) { return a; }, app::app_io::FailReason::Unkown),
      index, pivot, {/*"b_m_2_approx", "a_nodes_2_approx", "c_m/n"*/},
      [&](const auto& entry, const auto& virtual_func) -> Any {
        for (auto [k, v] : entry) {
          std::cout << k << " : " << v.to_string() << std::endl;
        }
        if (virtual_func == "b_m_2_approx") {
          for (const auto& d : data2) {
            if (d.hypergraph().name() == entry.at("hypergraph_name").to_string() &&
                d.run_config().short_name() == "2+dfs_m") {
              return ((double)d.algorithm_run_informations()[0].edge_count()) /
                     ((double)d.hypergraph().edge_count());
            }
          }
        }
        if (virtual_func == "a_nodes_2_approx") {
          for (const auto& d : data2) {
            if (d.hypergraph().name() == entry.at("hypergraph_name").to_string() &&
                d.run_config().short_name() == "2+dfs_m") {
              return ((double)d.algorithm_run_informations()[0].node_count()) /
                     ((double)d.hypergraph().node_count());
            }
          }
        }
        if (virtual_func == "c_m/n") {
          for (const auto& d : data2) {
            if (d.hypergraph().name() == entry.at("hypergraph_name").to_string() &&
                d.run_config().short_name() == "2+dfs_m") {
              return ((double)d.hypergraph().edge_count()) / ((double)d.hypergraph().node_count());
            }
          }
        }
        return {};
      });
  std::ofstream output(vis.folder_name() + "/" + vis.file_prefix() + "_table_" +
                       std::to_string(cap) + "_" + sort + ".tex");
  std::function<bool(Any, Any)> compare = [](Any a, Any b) { return a < b; };
  if (!bool_params["minimization_problem"]) {
    compare = [](Any a, Any b) { return a > b; };
  }
  output << pivot_table
                .to_table("size", bool_params["normalize_rows_max"], true, vis,
                          [&](auto a) {
                            std::vector<std::set<std::string>> res;
                            Any min = 0.0;
                            Any max = 0.0;
                            bool first = true;
                            for (const auto& [name, v] : a) {
                              if ((string_params["ignore_min"] != "") &&
                                  name.at("runConfig_shortName").to_string() >=
                                      string_params["ignore_min"]) {
                                continue;
                              }
                              if (first && !v.isError()) {
                                first = false;
                                min = v;
                                max = v;
                              }
                              if (compare(v, min) && !v.isError()) {
                                min = v;
                              }
                              if (v > max && !v.isError()) {
                                max = v;
                              }
                            }
                            for (auto& [name, v] : a) {
                              if ((string_params["ignore_min"] != "") &&
                                  name.at("runConfig_shortName").to_string() >=
                                      string_params["ignore_min"]) {
                                res.push_back({});
                                continue;
                              }
                              if (v.isError()) {
                                res.push_back({"error"});
                              } else if (v >= max && bool_params["max_bf"]) {
                                res.push_back({"max"});
                              } else if (v == min && !bool_params["max_bf"]) {
                                res.push_back({"min"});
                              } else {
                                res.push_back({});
                              }
                            }
                            return res;
                          })
                .to_latex("Caption", "table:def", pivot_table.colspec());
  std::cout << "Wrote "
            << vis.folder_name() + "/" + vis.file_prefix() + "_table_" + std::to_string(cap) + "_" +
                   sort + ".tex"
            << std::endl;
  return absl::OkStatus();
}
std::string rename_or_default(std::string s, auto map) {
  auto& ref = map[s];
  if (ref != "") {
    return ref;
  }
  return s;
}
absl::Status VisualisationTool::stats_from_debug_table_internal(
    Visualisation vis, std::vector<Result>& data2, std::vector<FailedExperiment>& failed,
    std::string sort, int64_t cap) {
  std::map<std::string, std::map<std::string, Any>> properties_by_hname;
  auto rlabels = vis.rename_labels();
  for (auto d : data2) {
    auto h_name = d.hypergraph().name();

    for (auto [k, v] : d.algorithm_run_informations()[d.algorithm_run_informations_size() - 1]
                           .debug_information()
                           .int64_info()) {
      properties_by_hname[rename_or_default(k, rlabels)][h_name] = v;
    }
    for (auto [k, v] : d.algorithm_run_informations()[d.algorithm_run_informations_size() - 1]
                           .debug_information()
                           .double_info()) {
      properties_by_hname[rename_or_default(k, rlabels)][h_name] = v;
    }
    for (auto [k, v] : d.algorithm_run_informations()[d.algorithm_run_informations_size() - 1]
                           .debug_information()
                           .bool_info()) {
      properties_by_hname[rename_or_default(k, rlabels)][h_name] = v;
    }
    for (auto [k, v] : d.algorithm_run_informations()[d.algorithm_run_informations_size() - 1]
                           .debug_information()
                           .string_info()) {
      properties_by_hname[rename_or_default(k, rlabels)][h_name] = v;
    }
  }
  auto display_labels = vis.display_labels();
  std::ofstream output(vis.folder_name() + "/" + vis.file_prefix() + "_debug_stats_table_" +
                       std::to_string(cap) + "_" + sort + ".tex");
  for (auto [k, map] : properties_by_hname) {
    output << " & " << rename_or_default(k, display_labels);
  }
  output << std::endl;
  if (properties_by_hname.size() == 0) {
    return absl::NotFoundError("no debug data found");
  }
  py::object lambda_func = py::eval("lambda x: x");
  auto string_params = vis.string_params();
  if (auto lsource = string_params["hypergraph_name_lambda"]; lsource != "") {
    lambda_func = py::eval(lsource);
  }
  for (auto [h, v] : properties_by_hname.begin()->second) {
    output << (lambda_func(h).cast<std::string>());
    for (auto [k, map] : properties_by_hname) {
      output << " & " << map[h].to_string();
    }
    output << "\\\\" << std::endl;
  }
  std::cout << "Written "
            << vis.folder_name() + "/" + vis.file_prefix() + "_debug_stats_table_" +
                   std::to_string(cap) + "_" + sort + ".tex"
            << std::endl;
  return absl::OkStatus();
}
absl::Status VisualisationTool::stats_table_internal(Visualisation vis, std::vector<Result>& data2,
                                                     std::vector<FailedExperiment>& failed,
                                                     const std::set<std::string> index,
                                                     const std::set<std::string> pivot,
                                                     std::string sort, int64_t cap) {
  auto string_params = vis.string_params();
  auto bool_params = vis.bool_params();
  std::vector<Result> data = data2;
  data = filter_exactness(data, vis);

  GroupBy<Result, std::map<std::string, Any>> grouped(
      data, [&](Result result) -> std::map<std::string, Any> {
        std::map<std::string, Any> res1;
        for (auto i : index) {
          res1.insert(std::make_pair(i, performance_characteristics.at(i)(result)));
        }
        return res1;
      });
  GroupBy<FailedExperiment, std::map<std::string, Any>> failed_group(
      failed, [&](FailedExperiment result) -> std::map<std::string, Any> {
        std::map<std::string, Any> res1;
        for (auto i : index) {
          res1.insert(std::make_pair(i, performance_characteristics_failed.at(i)(result)));
        }
        return {res1};
      });
  std::set<std::map<std::string, Any>> transformed_pivot;
  for (auto p : pivot) {
    transformed_pivot.insert({{"entry", p}});
  }
  std::map<std::map<std::string, Any>, GroupByEvaluator<Any>> evals;
  std::map<std::map<std::string, Any>, GroupByEvaluator<app::app_io::FailReason>> evals_fail;

  for (const auto& t : transformed_pivot) {
    evals[t] = AvgEvaluator<Any>();
    evals_fail[t] = FailEvaluator<app::app_io::FailReason>();
  }
  if (string_params["mean"] == "max") {
    for (const auto& t : transformed_pivot) {
      evals[t] = MaxEvaluator<Any>();
    }
  }
  if (string_params["mean"] == "min") {
    for (const auto& t : transformed_pivot) {
      evals[t] = MinEvaluator<Any>();
    }
  }
  if (string_params["mean"] == "geometric") {
    for (const auto& t : transformed_pivot) {
      evals[t] = GeomeanLogEvaluator<Any>();
    }
  }
  auto cnt = grouped.count();
  for (auto [k, v] : cnt) {
    for (auto [k1, k2] : k) {
      std::cout << k1 << " " << k2.to_string() << std::endl;
    }
    std::cout << ": " << v << std::endl;
  }
  PivotTable pivot_table(
      grouped.agg_many<Any>(
          transformed_pivot,
          [&](Result d, auto elem) {
            return performance_characteristics.at(elem["entry"].to_string())(d);
          },
          evals),
      failed_group.agg_many<app::app_io::FailReason>(
          transformed_pivot, [](auto FR, auto) { return FR.reason(); }, evals_fail),
      index, pivot);
  std::ofstream output(vis.folder_name() + "/" + vis.file_prefix() + "_stats_table_" +
                       std::to_string(cap) + "_" + sort + ".tex");

  output << pivot_table
                .to_table("size", bool_params["normalize_rows_max"], false, vis,
                          [&](auto a) {
                            std::vector<std::set<std::string>> res;
                            Any min = 0.0;
                            Any max = 0.0;
                            bool first = true;
                            for (const auto& [name, v] : a) {
                              if ((string_params["ignore_min"] != "") &&
                                  name.at("runConfig_shortName").to_string() >=
                                      string_params["ignore_min"]) {
                                continue;
                              }
                              if (first && !v.isError()) {
                                first = false;
                                min = v;
                                max = v;
                              }
                              if (v < min && !v.isError()) {
                                min = v;
                              }
                              if (v > max && !v.isError()) {
                                max = v;
                              }
                            }
                            for (auto& [name, v] : a) {
                              if ((string_params["ignore_min"] != "") &&
                                  name.at("runConfig_shortName").to_string() >=
                                      string_params["ignore_min"]) {
                                res.push_back({});
                                continue;
                              }
                              if (v.isError()) {
                                res.push_back({"error"});
                              } else if (v >= max && bool_params["max_bf"]) {
                                res.push_back({"max"});
                              } else if (v <= min && !bool_params["max_bf"]) {
                                res.push_back({"min"});
                              } else {
                                res.push_back({});
                              }
                            }
                            return res;
                          })
                .to_latex("Caption", "table:def", pivot_table.colspec());
  std::cout << "Wrote "
            << vis.folder_name() + "/" + vis.file_prefix() + "_stats_table_" + std::to_string(cap) +
                   "_" + sort + ".tex"
            << std::endl;
  return absl::OkStatus();
}
absl::Status VisualisationTool::write_csv_internal(Visualisation vis, std::vector<Result>& data,
                                                   std::vector<FailedExperiment>& failed,
                                                   std::string sort, int cap) {
  GroupBy<Result, std::array<std::string, 2>> groupby_instance_run_config(
      data, [](Result result) -> std::array<std::string, 2> {
        return {result.hypergraph().name(), result.run_config().short_name()};
      });
  auto avg = groupby_instance_run_config.avg<double>([&](auto r) -> double {
    return performance_characteristics.at(vis.performance_characteristic())(r).to_double();
  });
  std::ofstream file(vis.folder_name() + "/" + vis.file_prefix() + "_" + std::to_string(cap) + "_" +
                     sort + ".csv");
  file << "hgr_name,method,value" << std::endl;
  for (const auto& [k, v] : avg) {
    file << k[0] << "," << k[1] << "," << v.getValue() << std::endl;
  }
  std::cout << "Wrote "
            << vis.folder_name() + "/" + vis.file_prefix() + "_" + std::to_string(cap) + "_" +
                   sort + ".csv"
            << std::endl;
  return absl::OkStatus();
}
absl::Status VisualisationTool::categorial_bar_plot_internal(Visualisation vis,
                                                             std::vector<Result>& data,
                                                             std::vector<FailedExperiment>& failed,
                                                             std::string sort, int cap) {
  auto bool_params = vis.bool_params();
  auto string_params = vis.string_params();

  auto double_params = vis.double_params();
  auto display_labels = vis.display_labels();

  data = filter_exactness(data, vis);
  GroupBy<Result, std::array<std::string, 2>> grouped(
      data, [](Result r) -> std::array<std::string, 2> {
        return {r.run_config().short_name(), r.hypergraph().name()};
      });
  auto gr_avg = grouped.avg<double>([&](Result res) {
    return performance_characteristics.at(vis.performance_characteristic())(res).to_double();
  });
  GroupBy<typename decltype(gr_avg)::iterator::value_type, std::string> grouped_by_hname(
      gr_avg, [](auto res) { return res.first[1]; });
  GroupBy<typename decltype(gr_avg)::iterator::value_type, std::string> grouped_by_name(
      gr_avg, [](auto res) { return res.first[0]; });
  auto hypergraph_c = grouped_by_hname.max_count();

  std::set<std::string> in_all = grouped_by_hname
                                     .having([&](auto group, const auto& data) {
                                       bool zero = false;
                                       for (const auto& [_, s] : data) {
                                         if (s == 0.0 || std::isnan(s.getValue())) {
                                           zero = true;
                                         }
                                       }
                                       return !zero && (data.size() == hypergraph_c);
                                     })
                                     .keys();
  std::set<std::string> not_in_all =
      grouped_by_hname
          .having([&](auto group, const auto& data) { return (data.size() != hypergraph_c); })
          .keys();
  for (auto& c : not_in_all) {
    std::cout << c << std::endl;
  }
  std::cout << in_all.size() << " kept" << std::endl;
  GroupBy<Result, std::pair<std::string, std::string>> grouped2(data, [](auto r) {
    int category = 0;
    if (absl::StrContains(r.hypergraph().name(), "out50")) {
      category = 2;
    }
    return std::make_pair(r.run_config().short_name(), r.hypergraph().name());
  });

  auto results =
      grouped2.having([&](auto d, const auto& v) { return in_all.contains(d.second); })
          .geo_mean_log<double>([&](auto r) -> double {
            return performance_characteristics.at(vis.performance_characteristic())(r).to_double();
          });
  std::vector<std::pair<const std::pair<std::string, std::string>, double>> res_v;
  std::copy(results.begin(), results.end(), std::back_inserter(res_v));
  GroupBy<std::pair<const std::pair<std::string, std::string>, double>,
          std::pair<std::string, std::string>>
      gr3(res_v, [](auto val) {
        std::string category = "Lex50";
        if (absl::StrContains(val.first.second, "top50")) {
          category = "Lex+Top50";
        } else if (absl::StrContains(val.first.second, "out50")) {
          category = "Lex+Out50";
        }
        auto s = val.first.first;
        return std::make_pair<std::string, std::string>(std::move(s), std::move(category));
      });
  auto results3 = gr3.geo_mean_log<double>([](auto v) { return v.second; });
  auto [fig, ax] = plt.subplots();
  fig.unwrap().attr("set_figwidth")(*Args(double_params["fig_width_inch"]));
  fig.unwrap().attr("set_figheight")(*Args(double_params["fig_height_inch"]));
  double multiplier = 0;
  double width = 0.5;
  std::vector<std::string> labels = {"Lex", "Lex+Out50", "Lex+Top50"};
  std::map<std::string, std::vector<double>> results_2;
  std::vector<std::string> labels_;
  std::vector<double> offs;
  std::set<std::string> hnames;
  int k2 = 0;
  if (bool_params["grouped_by_top"]) {
    results = results3;
  }
  for (auto [k, v] : results) {
    std::cout << k.first << " " << k.second << " " << v << std::endl;
    results_2[k.second].push_back(v);
    if (!hnames.contains(k.first)) {
      hnames.insert(k.first);
      auto& titel = display_labels[k.first];
      if (titel == "") {
        titel = k.first;
      }
      labels_.push_back(titel);
      offs.push_back(0.33 + k2 * width * 4);
      k2++;
    }
  }
  for (auto [category, v] : results_2) {
    auto offset = width * multiplier;
    std::vector<double> ofs2;
    for (int i = 0; i < k2; i++) {
      ofs2.push_back(i * 2 + offset);
    }
    std::cout << category << " " << labels.size() << " " << v.size() << std::endl;
    auto rects = ax.bar(Args(ofs2, v, width), Kwargs("label"_a = category));
    multiplier++;
  }
  ax.set_xticks(Args(offs, labels_) /*, Kwargs("rotation"_a = 90.0)*/);
  ax.legend();
  ax.set_title(Args(prepare_title(vis, cap, grouped.max_count())));
  if (bool_params["y_log_scale"]) {
    ax.set_yscale(Args("log"));
  }
  std::string legend_pos = string_params["legend_pos"] == "" ? "best" : string_params["legend_pos"];
  ax.legend(Args(), Kwargs("loc"_a = legend_pos));
  fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_catbar_" + std::to_string(cap) +
                   "_" + sort + ".pdf"),
              Kwargs("bbox_inches"_a = "tight"));
  fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_catbar_" + std::to_string(cap) +
                   "_" + sort + ".pgf"),
              Kwargs("bbox_inches"_a = "tight"));
  save_textproto(vis.folder_name() + "/" + vis.file_prefix() + "_catbar_" + std::to_string(cap) +
                     "_" + sort + ".textproto",
                 vis);
  std::cout << "Plotted "
            << vis.folder_name() + "/" + vis.file_prefix() + "_catbar_" + std::to_string(cap) +
                   "_" + sort + ".pdf"
            << std::endl;
  std::cout << "Plotted "
            << vis.folder_name() + "/" + vis.file_prefix() + "_catbar_" + std::to_string(cap) +
                   "_" + sort + ".pgf"
            << std::endl;
  return absl::OkStatus();
}
absl::Status VisualisationTool::violin_plot_internal(Visualisation vis, std::vector<Result>& data,
                                                     std::vector<FailedExperiment>& failed,
                                                     std::string sort, int cap) {
  auto bool_params = vis.bool_params();
  auto double_params = vis.double_params();

  data = filter_exactness(data, vis);
  GroupBy<Result, std::string> grouped(
      data, [](Result r) -> std::string { return r.run_config().short_name(); });
  auto vdata = grouped.transform_all<std::pair<Any, int>>([&](auto v) {
    auto first = performance_characteristics.at(vis.performance_characteristic())(v);
    int category = 0;
    if (absl::StrContains(v.hypergraph().name(), "descending")) {
      category = 1;
    } else if (absl::StrContains(v.hypergraph().name(), "ascending")) {
      category = 2;
    }
    return std::make_pair(first, category);
  });
  std::map<std::string, std::vector<double>> plot_data;
  std::vector<std::string> datasets;
  std::vector<int> ticks;
  int i = 0;
  for (const auto& [k, v] : vdata) {
    std::vector<double> vv;
    ticks.push_back(i++);
    datasets.push_back(k);
    // std::transform(v.begin(), v.end(), std::back_inserter(vv), [&](auto t) -> std::vector<double>
    // {
    //   return {i - 1.0, t.first.to_double(), (double)t.second};
    // });
    // plot_data.push_back(vv);
    for (auto t : v) {
      plot_data["y"].push_back(i - 1.0);
      plot_data["x"].push_back(t.first.to_double());
      plot_data["category"].push_back(t.second);
    }
  }
  auto [fig, ax] = plt.subplots();
  fig.unwrap().attr("set_figwidth")(*Args(double_params["fig_width_inch"]));
  fig.unwrap().attr("set_figheight")(*Args(double_params["fig_height_inch"]));
  seaborn.attr("stripplot")(*Args(plot_data),
                            **Kwargs("x"_a = "y", "y"_a = "x", "hue"_a = "category",
                                     "log_scale"_a = bool_params["y_log_scale"]));
  ax.set_xticks(Args(ticks));
  ax.set_xticklabels(Args(datasets));
  fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_violin_" + std::to_string(cap) +
                   "_" + sort + ".pdf"),
              Kwargs("bbox_inches"_a = "tight"));
  save_textproto(vis.folder_name() + "/" + vis.file_prefix() + "_violin_" + std::to_string(cap) +
                     "_" + sort + ".textproto",
                 vis);
  // mod.attr("violin")
  std::cout << "Plotted " << vis.folder_name() << "/" + vis.file_prefix() << "_violin_"
            << std::to_string(cap) << "_" << sort << ".pdf" << std::endl;
  return absl::OkStatus();
}
absl::Status VisualisationTool::stats_internal(Visualisation vis, std::vector<Result>& data,
                                               std::vector<FailedExperiment>& failed,
                                               std::string sort, int cap) {
  data = filter_exactness(data, vis);
  GroupBy<Result, std::array<std::string, 2>> grouped(
      data, [](Result r) -> std::array<std::string, 2> {
        return {r.run_config().short_name(), r.hypergraph().name()};
      });
  auto gr_avg = grouped.avg<double>([&](Result res) {
    return performance_characteristics.at(vis.performance_characteristic())(res).to_double();
  });
  // std::map<std::string, int> cnts;
  // for (auto [c, val] : gr_avg) {
  //   std::cout << c[0] << " " << c[1] << " " << val << std::endl;
  //   cnts[c[0]]++;
  // }
  // for (auto [k, v] : cnts) {
  //   std::cout << k << " " << v << std::endl;
  // }
  auto grouped_by_exact_count =
      GroupBy<Result, std::string>(
          data, [](Result r) -> std::string { return r.run_config().short_name(); })
          .agg<int64_t>(
              [](Result r) -> int64_t {
                return r.algorithm_run_informations_size() > 0 &&
                       r.algorithm_run_informations()[r.algorithm_run_informations_size() - 1]
                           .is_exact();
              },
              [](int64_t a, int64_t b) { return a + b; }, [](int64_t a, size_t) { return a; }, 0);
  GroupBy<typename decltype(gr_avg)::iterator::value_type, std::string> grouped_by_hname(
      gr_avg, [](auto res) { return res.first[1]; });
  GroupBy<typename decltype(gr_avg)::iterator::value_type, std::string> grouped_by_name(
      gr_avg, [](auto res) { return res.first[0]; });
  auto hypergraph_c = grouped_by_hname.max_count();
  for (auto c : grouped_by_name.keys()) {
    std::cout << c << " " << grouped_by_name.count()[c] << std::endl;
  }
  std::cout << hypergraph_c << std::endl;

  std::set<std::string> in_all = grouped_by_hname
                                     .having([&](auto group, const auto& data) {
                                       bool zero = false;
                                       for (const auto& [_, s] : data) {
                                         if (s == 0.0) {
                                           zero = true;
                                         }
                                       }
                                       return !zero && (data.size() == hypergraph_c);
                                     })
                                     .keys();
  std::set<std::string> not_in_all =
      grouped_by_hname
          .having([&](auto group, const auto& data) { return (data.size() != hypergraph_c); })
          .keys();
  for (auto& c : not_in_all) {
    std::cout << c << std::endl;
  }
  auto filtered = grouped_by_name.where([&](const auto& d) { return in_all.contains(d.first[1]); });
  auto result = filtered.geo_mean_log<double>([&](auto res) { return res.second.getValue(); });
  auto string_params = vis.string_params();
  if (string_params["type"] == "max") {
    result = filtered.max<double>([&](auto res) { return res.second.getValue(); });
  }

  auto min = std::min_element(result.begin(), result.end(),
                              [](auto a, auto b) { return a.second < b.second; });
  auto bool_params = vis.bool_params();
  if (!bool_params[kMinimizationProblem]) {
    min = std::max_element(result.begin(), result.end(),
                           [](auto a, auto b) { return a.second < b.second; });
  }
  std::ofstream output(vis.folder_name() + "/" + vis.file_prefix() + "_stats_" +
                       std::to_string(cap) + "_" + sort + ".txt");
  std::ofstream output_tex(vis.folder_name() + "/" + vis.file_prefix() + "_stats_" +
                           std::to_string(cap) + "_" + sort + ".tex");
  output << in_all.size() << " data points" << std::endl;
  for (auto [k, v] : result) {
    output << "RunConfig: " << k << " "
           << (string_params["type"] == "" ? std::string("GeoMean") : string_params["type"]) << ": "
           << v << " (" << v / min->second << ",e: " << grouped_by_exact_count[k] << ")"
           << std::endl;
  }
  std::vector<std::pair<std::string, double>> result_sorted;
  std::transform(result.begin(), result.end(), std::back_inserter(result_sorted),
                 [](auto a) { return std::make_pair(a.first, a.second); });
  std::sort(result_sorted.begin(), result_sorted.end(),
            [](auto a, auto b) { return a.second < b.second; });
  output_tex << std::fixed << std::setprecision(2);
  for (auto [k, v] : result_sorted) {
    output_tex << k << " & " << v << " & " << v / min->second << "\\\\" << std::endl;
  }
  output_tex << "\\bottomrule" << std::endl;
  std::cout << "Wrote "
            << vis.folder_name() + "/" + vis.file_prefix() + "_stats_" + std::to_string(cap) + "_" +
                   sort + ".txt"
            << std::endl;
  std::cout << "Wrote "
            << vis.folder_name() + "/" + vis.file_prefix() + "_stats_" + std::to_string(cap) + "_" +
                   sort + ".tex"
            << std::endl;
  return absl::OkStatus();
}
absl::Status VisualisationTool::table(std::vector<ExperimentResult>& data, Visualisation vis) {
  auto bool_params = vis.bool_params();
  std::set<std::string> index, pivot;
  auto string_params = vis.string_params();

  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "index" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      index.insert(string_params[s.str()]);
    }
  }
  if (index.empty()) {
    index = {"size", "hypergraph_name"};
  }
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "pivot" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      pivot.insert(string_params[s.str()]);
    }
  }
  if (pivot.empty()) {
    pivot = {"runConfig_shortName"};
  }
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = table_internal(vis, filtered, error_points, index, pivot, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = table_internal(vis, data_points, error_points,
                                        index,  //{"hypergraph_name_sort"}
                                                /*{"hypergraph_name", "hypergraph_name_sort",
                                                 "hypergraph_nedge", "hypergraph_nnode", "m/n"}*/
                                        pivot);
          !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::stats_table(std::vector<ExperimentResult>& data,
                                            Visualisation vis) {
  auto bool_params = vis.bool_params();
  std::set<std::string> index, pivot;
  auto string_params = vis.string_params();

  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "index" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      index.insert(string_params[s.str()]);
    }
  }
  if (index.empty()) {
    index = {"size", "hypergraph_name"};
  }
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "pivot" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      pivot.insert(string_params[s.str()]);
    }
  }
  if (pivot.empty()) {
    pivot = {"max_d", "average_d", "max_node_degree", "m", "n"};
  }
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = stats_table_internal(vis, filtered, error_points, index, pivot, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = stats_table_internal(vis, data_points, error_points,
                                              index,  //{"hypergraph_name_sort"}
                                                      /*{"hypergraph_name", "hypergraph_name_sort",
                                                       "hypergraph_nedge", "hypergraph_nnode", "m/n"}*/
                                              pivot);
          !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::stats(std::vector<ExperimentResult>& data, Visualisation vis) {
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = stats_internal(vis, filtered, error_points, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = stats_internal(vis, data_points, error_points, sort); !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::stats_debug(std::vector<ExperimentResult>& data,
                                            Visualisation vis) {
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = stats_from_debug_table_internal(vis, filtered, error_points, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = stats_from_debug_table_internal(vis, data_points, error_points, sort);
          !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::write_csv(std::vector<ExperimentResult>& data, Visualisation vis) {
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = write_csv_internal(vis, filtered, error_points, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = write_csv_internal(vis, data_points, error_points, sort); !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}

absl::Status VisualisationTool::categorial_bar_plot(std::vector<ExperimentResult>& data,
                                                    Visualisation vis) {
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = categorial_bar_plot_internal(vis, filtered, error_points, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = categorial_bar_plot_internal(vis, data_points, error_points, sort);
          !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
std::vector<double> to_vec(auto map, bool sort = false, const std::set<std::string>& remove = {}) {
  std::vector<double> vec;
  for (const auto& s : remove) {
    map.erase(s);
  }
  std::transform(map.begin(), map.end(), std::back_inserter(vec),
                 [](auto a) { return a.second.to_double(); });
  if (sort) {
    std::sort(vec.begin(), vec.end());
  }
  return vec;
}
std::pair<std::set<std::string>, std::set<std::string>> split(const auto& map, double first_set) {
  std::vector<std::string> res;
  std::transform(map.begin(), map.end(), std::back_inserter(res), [](auto a) { return a.first; });
  std::random_shuffle(res.begin(), res.end());
  auto it = res.begin() + (size_t)(first_set * res.size());
  std::set<std::string> set_a, set_b;
  std::copy(res.begin(), it, std::inserter(set_a, set_a.begin()));
  std::copy(it, res.end(), std::inserter(set_b, set_b.begin()));
  std::cout << "Split: " << set_a.size() << " " << set_b.size() << std::endl;
  return std::make_pair(set_a, set_b);
}
absl::Status VisualisationTool::jointplot_plot_internal(Visualisation vis,
                                                        std::vector<Result>& data,
                                                        std::vector<FailedExperiment>& failed,
                                                        std::string sort, int cap) {
  auto bool_params = vis.bool_params();
  auto string_params = vis.string_params();

  auto double_params = vis.double_params();
  auto display_labels = vis.display_labels();

  data = filter_exactness(data, vis);
  GroupBy<Result, std::pair<std::string, std::string>> grouped(data, [](Result r) {
    return std::make_pair(r.run_config().short_name(), r.hypergraph().name());
  });
  std::map<std::string, std::map<std::string, Any>> hypergraph_properties,
      hypergraph_properties2;  // first plot type, second instance
  for (const auto& t : data) {
    for (const auto& as : t.algorithm_run_informations()) {
      for (const auto& [k, v] : as.debug_information().string_info()) {
        hypergraph_properties[k][t.hypergraph().name()] = v;
      }
      for (const auto& [k, v] : as.debug_information().int64_info()) {
        hypergraph_properties[k][t.hypergraph().name()] = v;
      }
      for (const auto& [k, v] : as.debug_information().double_info()) {
        hypergraph_properties[k][t.hypergraph().name()] = v;
      }
      for (const auto& [k, v] : as.debug_information().bool_info()) {
        hypergraph_properties[k][t.hypergraph().name()] = v;
      }
    }
  }
  auto results = grouped.geo_mean_log<double>([&](auto r) -> double {
    return performance_characteristics.at(vis.performance_characteristic())(r).to_double();
  });
  auto [fig, ax] = plt.subplots();
  fig.unwrap().attr("set_figwidth")(*Args(double_params["fig_width_inch"]));
  fig.unwrap().attr("set_figheight")(*Args(double_params["fig_height_inch"]));
  std::vector<std::string> labels_;
  std::vector<double> offs;
  std::map<std::string, double> x_values;
  std::map<std::string, std::map<std::string, double>> yvalues;
  int k2 = 0;
  // std::map<std::string, double> best_per_instance;
  for (auto [v, d] : hypergraph_properties["average_d"]) {
    hypergraph_properties["pins"][v] = d.to_double() * hypergraph_properties["m"][v].to_double();
  }
  for (const auto& [k, v] : results) {
    if (k.first == "stats") {
      continue;
    }
    std::cout << k.first << " " << k.second << " " << v << std::endl;
    hypergraph_properties2[k.first][k.second] = (v);
  }
  std::vector<std::string> hues;
  // auto best_per_instance = hypergraph_properties2["seq_weight_rnd"];
  std::map<std::string, std::vector<double>> plot_data;
  for (const auto& [k2, v2] : hypergraph_properties2) {
    std::cout << k2 << std::endl;

    for (const auto& [k, values] : hypergraph_properties) {
      std::transform(values.begin(), values.end(), std::back_inserter(plot_data[k]),
                     [&](auto a) { return a.second.to_double(); });
    }
    for (auto [b, v3] : v2) {
      plot_data[vis.performance_characteristic()].push_back(v3.to_double());
    }
    std::transform(v2.begin(), v2.end(), std::back_inserter(hues), [&](auto a) {
      auto s = display_labels[k2];
      if (s == "") {
        s = k2;
      }
      return s;
    });
  }
  std::cout << hues.size() << " entries" << std::endl;
  std::cout << plot_data[string_params["x_value"]].size() << std::endl;
  std::cout << plot_data[string_params["y_value"]].size() << std::endl;
  py::dict plt_data2;

  for (auto [k, v] : plot_data) {
    plt_data2[k.c_str()] = v;
  }
  plt_data2["hues"] = hues;
  py::object plt_data = pandas.attr("DataFrame")(*Args(plt_data2));
  // seaborn.attr("jointplot")(
  //     *Args(plot_data), **Kwargs("kind"_a = string_params["kind"], "x"_a =
  //     string_params["x_value"],
  //                                "y"_a = string_params["y_value"]));  //, "hue"_a = hues
  seaborn.attr("scatterplot")(
      *Args(), **Kwargs("x"_a = string_params["x_value"], "y"_a = string_params["y_value"],
                        "hue"_a = std::string("hues"), "style"_a = std::string("hues"),
                        "data"_a = plt_data));
  std::map<std::string, std::vector<double>> plt_data_;

  // plt_data_["x"] = to_vec(hypergraph_properties[string_params["x_value"]]);
  // plt_data_["y"] = to_vec(hypergraph_properties2[string_params["fit"]]);
  // py::object plt_data4 = pandas.attr("DataFrame")(*Args(plt_data_));
  if (string_params["fit"] != "") {
    // seaborn.attr("lmplot")(*Args(), **Kwargs("data"_a = plt_data4));
    //  "logx"_a = true,
    //"ci"_a = 99
    //  ax.set_xticks(Args(offs, labels_) /*, Kwargs("rotation"_a = 90.0) */);
    //  py::eval("from sklearn.linear_model import LinearRegression");
    auto [test, train] =
        split(hypergraph_properties[string_params["x_value"]], double_params["split"]);
    py::object regr = skl.attr("LinearRegression")().attr("fit")(*Args(
        np.attr("array")(to_vec(hypergraph_properties[string_params["x_value"]], false, train))
            .attr("reshape")(*Args(-1, 1)),
        np.attr("array")(to_vec(hypergraph_properties2[string_params["fit"]], false, train))));
    auto regr_plot = regr.attr("predict")(
        *Args(np.attr("array")(to_vec(hypergraph_properties[string_params["x_value"]], true, test))
                  .attr("reshape")(*Args(-1, 1))));
    std::cout << string_params["fit"] << " Score: "
              << regr
                     .attr("score")(*Args(
                         np.attr("array")(
                               to_vec(hypergraph_properties[string_params["x_value"]], false, test))
                             .attr("reshape")(*Args(-1, 1)),
                         np.attr("array")(
                             to_vec(hypergraph_properties2[string_params["fit"]], false, test))))
                     .cast<double>()
              << " test: "
              << to_vec(hypergraph_properties2[string_params["fit"]], false, test).size()
              << std::endl;
    plt.plot(Args(to_vec(hypergraph_properties[string_params["x_value"]], true, test), regr_plot),
             Kwargs("label"_a = string_params["fit_label"]));
  }
  // py::dict globals = py::globals();
  // globals["numpy"] = np;
  // py::object conf_func = py::eval(
  //     "lambda X_train, y_train,linreg: 1.96 * numpy.sqrt(sum((linreg.predict(X_train) - "
  //     "y_train)**2) / "
  //     "(len(y_train) - "
  //     "2))",
  //     globals);
  // double conf =
  //     conf_func(*Args(np.attr("array")(
  //                           to_vec(hypergraph_properties[string_params["x_value"]], false,
  //                           train))
  //                         .attr("reshape")(*Args(-1, 1)),
  //                     to_vec(hypergraph_properties2[string_params["fit"]], false, train), regr))
  //         .cast<double>();
  // std::cout << conf << std::endl;
  // ax.unwrap().attr("fill_between")(
  //     *Args(to_vec(hypergraph_properties[string_params["x_value"]], true, test),
  //           regr_plot.attr("__sub__")(*Args(conf)), regr_plot.attr("__sub__")(*Args(-conf))),
  //     **Kwargs("color"_a = "red", "alpha"_a = 0.3, "label"_a = "95% Confidence Interval"));
  // ax.legend();
  ax.set_title(Args(prepare_title(vis, cap, grouped.max_count())));
  ax.set_xlabel(Args(string_params["xlabel"]));
  ax.set_ylabel(Args(string_params["ylabel"]));

  if (bool_params["y_log_scale"]) {
    ax.set_yscale(Args("log"));
  }
  if (bool_params["x_log_scale"]) {
    ax.set_xscale(Args("log"));
  }
  std::string legend_pos = string_params["legend_pos"] == "" ? "best" : string_params["legend_pos"];
  ax.legend(Args(), Kwargs("loc"_a = legend_pos));
  plt.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_jointmap_" +
                   std::to_string(cap) + "_" + sort + ".pdf"),
              Kwargs("bbox_inches"_a = "tight"));
  save_textproto(vis.folder_name() + "/" + vis.file_prefix() + "_jointmap_" + std::to_string(cap) +
                     "_" + sort + ".textproto",
                 vis);
  std::cout << "Plotted "
            << vis.folder_name() + "/" + vis.file_prefix() + "_jointmap_" + std::to_string(cap) +
                   "_" + sort + ".pdf"
            << std::endl;
  return absl::OkStatus();
}
absl::Status VisualisationTool::jointplot_plot(std::vector<ExperimentResult>& data,
                                               Visualisation vis) {
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = jointplot_plot_internal(vis, filtered, error_points, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = jointplot_plot_internal(vis, data_points, error_points, sort);
          !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::scatter_debug_plot_internal(Visualisation vis,
                                                            std::vector<Result>& data,
                                                            std::vector<FailedExperiment>& failed,
                                                            std::string sort, int cap) {
  auto bool_params = vis.bool_params();
  auto string_params = vis.string_params();

  auto double_params = vis.double_params();
  auto display_labels = vis.display_labels();

  data = filter_exactness(data, vis);
  GroupBy<Result, std::pair<std::string, std::string>> grouped(data, [](Result r) {
    return std::make_pair(r.run_config().short_name(), r.hypergraph().name());
  });
  std::map<std::string, std::map<std::string, Any>> hypergraph_properties;
  for (const auto& t : data) {
    for (const auto& as : t.algorithm_run_informations()) {
      for (const auto& [k, v] : as.debug_information().string_info()) {
        hypergraph_properties[t.hypergraph().name()][k] = v;
      }
      for (const auto& [k, v] : as.debug_information().int64_info()) {
        hypergraph_properties[t.hypergraph().name()][k] = v;
      }
      for (const auto& [k, v] : as.debug_information().double_info()) {
        hypergraph_properties[t.hypergraph().name()][k] = v;
      }
      for (const auto& [k, v] : as.debug_information().bool_info()) {
        hypergraph_properties[t.hypergraph().name()][k] = v;
      }
    }
  }
  auto results = grouped.geo_mean_log<double>([&](auto r) -> double {
    return performance_characteristics.at(vis.performance_characteristic())(r).to_double();
  });
  auto [fig, ax] = plt.subplots();
  fig.unwrap().attr("set_figwidth")(*Args(double_params["fig_width_inch"]));
  fig.unwrap().attr("set_figheight")(*Args(double_params["fig_height_inch"]));
  std::vector<std::string> labels_;
  std::vector<double> offs;
  std::map<std::string, double> x_values;
  std::map<std::string, std::map<std::string, double>> yvalues;
  int k2 = 0;
  std::map<std::string, double> best_per_instance;

  for (const auto& [k, v] : results) {
    if (k.first == "stats") {
      continue;
    }
    std::cout << k.first << " " << k.second << " " << v << std::endl;
    yvalues[k.first][k.second] = (v);
    auto& b = best_per_instance[k.second];
    b = std::max(b, v);
    x_values[k.second] = hypergraph_properties[k.second][string_params["x_value"]].to_double();
  }
  std::vector<double> vecX;
  std::transform(x_values.begin(), x_values.end(), std::back_inserter(vecX),
                 [](auto a) { return a.second; });
  for (const auto& [k, values] : yvalues) {
    std::vector<double> vecY;
    std::transform(values.begin(), values.end(), std::back_inserter(vecY),
                   [&](auto a) { return a.second /*/ best_per_instance[a.first]*/; });
    ax.scatter(Args(vecX, vecY), Kwargs("label"_a = k));
  }
  // ax.set_xticks(Args(offs, labels_) /*, Kwargs("rotation"_a = 90.0) */);
  ax.legend();
  ax.set_title(Args(prepare_title(vis, cap, grouped.max_count())));
  if (bool_params["y_log_scale"]) {
    ax.set_yscale(Args("log"));
  }
  if (bool_params["x_log_scale"]) {
    ax.set_xscale(Args("log"));
  }
  std::string legend_pos = string_params["legend_pos"] == "" ? "best" : string_params["legend_pos"];
  ax.legend(Args(), Kwargs("loc"_a = legend_pos));
  fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_scatterdebug_" +
                   std::to_string(cap) + "_" + sort + ".pdf"),
              Kwargs("bbox_inches"_a = "tight"));
  save_textproto(vis.folder_name() + "/" + vis.file_prefix() + "_scatterdebug_" +
                     std::to_string(cap) + "_" + sort + ".textproto",
                 vis);
  std::cout << "Plotted "
            << vis.folder_name() + "/" + vis.file_prefix() + "_scatterdebug_" +
                   std::to_string(cap) + "_" + sort + ".pdf"
            << std::endl;
  return absl::OkStatus();
}
absl::Status VisualisationTool::scatter_debug_plot(std::vector<ExperimentResult>& data,
                                                   Visualisation vis) {
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = scatter_debug_plot_internal(vis, filtered, error_points, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = scatter_debug_plot_internal(vis, data_points, error_points, sort);
          !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::violin_plot(std::vector<ExperimentResult>& data,
                                            Visualisation vis) {
  auto transformed = transform_by_sort(data, vis);
  auto transformed_failed = transform_by_sort_failed(data, vis);
  for (auto& [sort, data_points] : transformed) {
    auto error_points = transformed_failed[sort];
    if (vis.capacity_size() > 0) {
      for (auto cap : vis.capacity()) {
        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = violin_plot_internal(vis, filtered, error_points, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
    } else {
      if (auto plotted = violin_plot_internal(vis, data_points, error_points, sort);
          !plotted.ok()) {
        return plotted;
      }
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::efficency(std::vector<ExperimentResult>& data, Visualisation vis) {
  std::cout << "ScatterPlotting " << data.size() << std::endl;
  auto double_params = vis.double_params();
  auto transformed = transform_by_sort(data, vis);

  for (auto& [sort, data_points] : transformed) {
    if (vis.capacity_size() > 0) {
      auto sub =
          mod.attr("subplots")(
                 **Args(), **Kwargs("ncols"_a = 2, "nrows"_a = 2,
                                    "figsize"_a = py::make_tuple(double_params["fig_width_inch"],
                                                                 double_params["fig_height_inch"]),
                                    "layout"_a = "constrained"))
              .cast<py::list>();
      auto fig = matplotlibcpp17::figure::Figure(sub[0]);
      auto axes = sub[1];

      int i = 0;
      for (auto cap : vis.capacity()) {
        auto ax = matplotlibcpp17::axes::Axes(
            *((py::object*)axes.cast<py::array>().attr("flatten")().cast<py::array>().data(i++)));

        decltype(data_points) filtered;
        std::copy_if(data_points.begin(), data_points.end(), std::back_inserter(filtered),
                     [cap](Result e) { return e.run_config().capacity() == cap; });
        auto plotted = efficency_internal(fig, ax, vis, filtered, sort, cap);
        if (!plotted.ok()) {
          return plotted;
        }
      }
      plt.legend();
      plt.savefig(
          Args(vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" + "_" + sort + ".png"));
      plt.savefig(
          Args(vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" + "_" + sort + ".pdf"),
          Kwargs("bbox_inches"_a = "tight"));
      save_textproto(
          vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" + "_" + sort + ".textproto",
          vis);
      std::cout << "Plotted "
                << (vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" + "_" + sort +
                    ".png")
                << std::endl
                << (vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" + "_" + sort +
                    ".pdf")
                << std::endl;
    } else {
      auto double_params = vis.double_params();
      auto fig = plt.figure(Args(),
                            Kwargs("figsize"_a = py::make_tuple(double_params["fig_width_inch"],
                                                                double_params["fig_height_inch"])));

      auto ax = fig.add_axes(Args(std::vector<double>({0.125, 0.175, 0.80, 0.75})));
      auto plotted = efficency_internal(fig, ax, vis, data_points, sort);
      if (!plotted.ok()) {
        return plotted;
      }
      ax.legend();
      ax.set_title(Args(sort));
      fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" +
                       std::to_string(0) + "_" + sort + ".png"));
      fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" +
                       std::to_string(0) + "_" + sort + ".pdf"),
                  Kwargs("bbox_inches"_a = "tight"));
      save_textproto(vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" +
                         std::to_string(0) + "_" + sort + ".textproto",
                     vis);
      std::cout << "Plotted "
                << (vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" +
                    std::to_string(0) + "_" + sort + ".png")
                << std::endl
                << (vis.folder_name() + "/" + vis.file_prefix() + "_efficency_" +
                    std::to_string(0) + "_" + sort + ".pdf")
                << std::endl;
    }
  }
  return absl::OkStatus();
}
absl::Status VisualisationTool::efficency_internal(matplotlibcpp17::figure::Figure& fig,
                                                   matplotlibcpp17::axes::Axes& ax,
                                                   Visualisation vis, std::vector<Result>& data,
                                                   std::string sort, int cap) {
  std::map<std::string, double> double_map;
  std::map<std::string, int64_t> int_map;
  for (auto& d : data) {
    for (auto [k, v] : d.algorithm_run_informations().at(0).debug_information().double_info()) {
      double_map[k] += v;
    }
    for (auto [k, v] : d.algorithm_run_informations().at(0).debug_information().int64_info()) {
      int_map[k] += v;
    }
  }
  std::cout << "Plotting " << sort << std::endl;
  auto double_params = vis.double_params();
  std::vector<std::string> bars = {"Time", "Effect"};
  double sum_runtime = 0.0;
  for (auto [k, v] : double_map) {
    sum_runtime += v;
  }
  double sum_impact = 0.0;
  for (auto [k, v] : int_map) {
    sum_impact += v;
  }
  std::vector<double> bottom{0.0, 0.0};
  for (auto [k, v] : double_map) {
    std::cout << k << "," << v << std::endl;
    std::vector<double> entries = {v / sum_runtime, ((double)int_map[k]) / sum_impact};
    ax.bar(Args(bars, entries, 0.25), Kwargs("bottom"_a = bottom, "label"_a = k));
    bottom[0] += entries[0];
    bottom[1] += entries[1];
  }
  ax.set_title(Args("b(v)=" + (cap == -1 ? "rnd" : std::to_string(cap))));
  return absl::OkStatus();
}
absl::Status VisualisationTool::scatter_plot_internal(Visualisation vis, std::vector<Result>& data,
                                                      std::string sort, int cap) {
  auto bool_params = vis.bool_params();
  auto double_params = vis.double_params();
  auto string_params = vis.string_params();

  std::function<double(double, double)> optimal = [](double a, double b) { return std::max(a, b); };
  std::function<double(double, double)> not_optimal = [](double a, double b) {
    return std::min(a, b);
  };
  std::function<double(double, double)> comparator = [](double a, double b) { return a > b; };
  if (bool_params[kMinimizationProblem]) {
    optimal = [](double a, double b) { return std::min(a, b); };
    not_optimal = [](double a, double b) { return std::max(a, b); };
    comparator = [](double a, double b) { return a < b; };
  }
  auto ratios_s = normalize_by_optimum(vis, data, sort, cap);

  if (!ratios_s.ok()) {
    return ratios_s.status();
  }
  auto [ratios_m, count] = std::move(ratios_s.value());

  // map is ordered by <
  std::map<std::string, std::array<std::vector<double>, 2>> y_axis;

  std::vector<double> x_axis;
  std::map<std::string, double> x_axis_by_name;
  std::function<double(Result)> selector = [](auto r) {
    auto& d = r.hypergraph();
    return d.edge_count() + d.node_count();
  };
  auto x_label = string_params["x_axis"];
  if (string_params["x_axis"] != "") {
    if (string_params["x_axis"] == "avg_degree") {
      x_label = "$\\rho$";
      selector = [](auto r) {
        auto& d = r.hypergraph();
        return ((double)d.edge_count()) / ((double)d.node_count());
      };
    } else if (string_params["x_axis"] == "edge_density") {
      selector = [](auto r) {
        auto& d = r.hypergraph();

        return 2.0 * ((double)d.edge_count()) / pow((double)d.node_count(), 2);
      };

    } else if (string_params["x_axis"] == "vertex_density") {
      selector = [](auto r) {
        auto& d = r.hypergraph();

        return ((double)d.node_count()) / ((double)d.node_count() * ((double)d.node_count() / 2.0));
      };
    } else if (string_params["x_axis"] == "min_out_degree") {
      x_label = "$d^\\star$";
      selector = [](auto r) { return r.size(); };
    } else if (string_params["x_axis"] == "edges") {
      selector = [](auto r) { return r.hypergraph().edge_count(); };
    } else {
      return absl::UnimplementedError(string_params["x_axis"]);
    }
  }
  for (const auto& d : data) {
    x_axis_by_name[d.hypergraph().name()] = selector(d);
  }

  std::transform(x_axis_by_name.begin(), x_axis_by_name.end(), std::back_inserter(x_axis),
                 [](auto a) { return a.second; });
  auto fig =
      plt.figure(Args(), Kwargs("figsize"_a = py::make_tuple(double_params["fig_width_inch"],
                                                             double_params["fig_height_inch"])));
  auto ax = fig.add_axes(Args(std::vector<double>({0.125, 0.175, 0.80, 0.75})));
  for (const auto& [algo, algo_data] : ratios_m) {
    std::vector<double> y_a, er_a, x_axis;
    std::transform(algo_data.begin(), algo_data.end(), std::back_inserter(y_a),
                   [](auto a) { return a.second.getValue(); });
    std::transform(algo_data.begin(), algo_data.end(), std::back_inserter(x_axis),
                   [&](auto a) { return x_axis_by_name[a.first]; });
    std::transform(algo_data.begin(), algo_data.end(), std::back_inserter(er_a),
                   [](auto a) { return a.second.stdev(); });

    ax.scatter(Args(x_axis, y_a), Kwargs("label"_a = algo));
  }
  auto least_optimal_value = x_axis.front();
  for (const auto& x : x_axis) {
    least_optimal_value = not_optimal(least_optimal_value, x);
  }
  ax.set_xscale(Args("log"));
  if (string_params["y_scale"] == "log") {
    ax.set_yscale(Args("log"));
  }
  ax.set_xlabel(Args(x_label));
  ax.set_ylabel(Args("$t_{rel}$"));
  std::string legend_pos = string_params["legend_pos"] == "" ? "best" : string_params["legend_pos"];
  ax.legend(Args(), Kwargs("loc"_a = legend_pos));

  ax.set_title(Args(prepare_title(vis, cap, count)));
  fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_scatter_profile_" +
                   std::to_string(cap) + "_" + sort + ".png"));
  fig.savefig(Args(vis.folder_name() + "/" + vis.file_prefix() + "_scatter_profile_" +
                   std::to_string(cap) + "_" + sort + ".pdf"),
              Kwargs("bbox_inches"_a = "tight"));
  save_textproto(vis.folder_name() + "/" + vis.file_prefix() + "_scatter_profile_" +
                     std::to_string(cap) + "_" + sort + ".textproto",
                 vis);
  std::cout << "Plotted "
            << (vis.folder_name() + "/" + vis.file_prefix() + "_scatter_profile_" +
                std::to_string(cap) + "_" + sort + ".png")
            << std::endl
            << (vis.folder_name() + "/" + vis.file_prefix() + "_scatter_profile_" +
                std::to_string(cap) + "_" + sort + ".pdf")
            << std::endl;
  plt.clf(Args(), Kwargs());
  // TODO add min/max settings
  return absl::OkStatus();
}
std::map<std::string, std::vector<FailedExperiment>> VisualisationTool::transform_by_sort_failed(
    std::vector<ExperimentResult>& data, Visualisation vis) {
  auto bool_params = vis.bool_params();
  bool minimization_problem = bool_params[kMinimizationProblem];
  bool group_by = bool_params["groupby_sort"];
  std::set<std::string> ignore_substr, ignore_exact, ignore_substr_hyper_name;
  auto string_params = vis.string_params();
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "ignore_substr" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      ignore_substr.insert(string_params[s.str()]);
    }
  }
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "ignore_substr_hypergraph_name" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      ignore_substr_hyper_name.insert(string_params[s.str()]);
    }
  }
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "ignore_exact" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      ignore_exact.insert(string_params[s.str()]);
    }
  }
  std::cout << "[INFO] IGNORING RULES "
            << ignore_substr.size() + ignore_exact.size() + ignore_substr_hyper_name.size()
            << std::endl;
  std::map<std::string, std::vector<FailedExperiment>> result;
  for (auto d : data) {
    for (auto r : d.failed()) {
      bool ignore = false;
      for (auto s : ignore_substr) {
        if (r.run_config().short_name().find(s) != std::string::npos) {
          ignore = true;
          break;
        }
      }
      for (auto s : ignore_exact) {
        if (r.run_config().short_name() == s) {
          ignore = true;
          break;
        }
      }
      for (const auto& s : ignore_substr_hyper_name) {
        if (r.hypergraph().name().find(s) != std::string::npos) {
          ignore = true;
          break;
        }
      }
      if (ignore) {
        continue;
      }
      result["all"].push_back(r);
      if (group_by) {
        result[r.hypergraph().sort()].push_back(r);
      }
    }
  }
  return result;
}
std::map<std::string, std::vector<Result>> VisualisationTool::transform_by_sort(
    std::vector<ExperimentResult>& data, Visualisation vis) {
  auto bool_params = vis.bool_params();
  bool minimization_problem = bool_params[kMinimizationProblem];
  bool group_by = bool_params["groupby_sort"];
  std::set<std::string> ignore_substr, ignore_exact, ignore_substr_hyper_name, must_contain,
      must_contain_short_name;
  auto string_params = vis.string_params();
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "shortname_must_contain" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      must_contain_short_name.insert(string_params[s.str()]);
    }
  }
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "hgr_must_contain" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      must_contain.insert(string_params[s.str()]);
    }
  }
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "ignore_substr" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      ignore_substr.insert(string_params[s.str()]);
    }
  }
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "ignore_substr_hypergraph_name" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      ignore_substr_hyper_name.insert(string_params[s.str()]);
    }
  }
  for (int i = 0; i < 10; i++) {
    std::stringstream s;
    s << "ignore_exact" << i;
    // std::cout << s.str() << std::endl;
    if (string_params[s.str()] != "") {
      ignore_exact.insert(string_params[s.str()]);
    }
  }
  std::cout << "[INFO] IGNORING RULES "
            << ignore_substr.size() + ignore_exact.size() + ignore_substr_hyper_name.size()
            << std::endl;
  std::map<std::string, std::vector<Result>> result;
  for (auto d : data) {
    for (auto r : d.results()) {
      bool ignore = false;
      for (auto s : must_contain) {
        if (!absl::StrContains(r.hypergraph().name(), s)) {
          ignore = true;
          break;
        }
      }
      for (auto s : must_contain_short_name) {
        if (!absl::StrContains(r.run_config().short_name(), s)) {
          ignore = true;
          break;
        }
      }
      for (auto s : ignore_substr) {
        if (r.run_config().short_name().find(s) != std::string::npos) {
          ignore = true;
          break;
        }
      }
      for (auto s : ignore_exact) {
        if (r.run_config().short_name() == s) {
          ignore = true;
          break;
        }
      }
      for (auto s : ignore_substr_hyper_name) {
        if (r.hypergraph().name().find(s) != std::string::npos) {
          ignore = true;
          break;
        }
      }
      if (ignore) {
        continue;
      }
      result["all"].push_back(r);
      if (group_by) {
        result[r.hypergraph().sort()].push_back(r);
      }
    }
  }
  return result;
}
}  // namespace plot
}  // namespace tools
}  // namespace dyn_delta_approx

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage:" << std::endl
              << "plot <path to experiment git> <visualisation files>" << std::endl;
    std::cerr << "Please supply more arguments" << std::endl;
    return 1;
  }
  bool install_deps = 1;
  if (install_deps) {
    std::cout << "Making sure every dep is installed..." << std::endl;

    // pybind11::scoped_interpreter guard{};
    // pybind11::exec(R"(
    //     import subprocess
    //     # Install the desired Python package using pip
    //     package_name = "matplotlib"
    //     subprocess.check_call(["python3" ,"-m" ,"ensurepip" ,"--default-pip"])
    //     subprocess.check_call(["python3","-m", "pip", "install", package_name])
    // )");
    system("python3 -m ensurepip --default-pip");
    system("python3 -m pip install matplotlib");
    system("python3 -m pip install seaborn scikit-learn");
  }
  for (int i = 2; i < argc; i++) {
    dyn_delta_approx::tools::plot::VisualisationTool vis(argv[i]);
    if (auto status = vis.plot(argv[1]); !status.ok()) {
      std::cerr << "[ERROR] Failed path:" << argv[i] << std::endl;
      std::cerr << "[ERROR] Status: " << status.code() << std::endl;
    } else {
      std::cout << "Done " << argv[i] << std::endl;
    }
  }
  return 0;
}
