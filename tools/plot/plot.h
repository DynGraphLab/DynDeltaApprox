#pragma once

#include <algorithm>
#include <any>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "Python.h"
#include "absl/status/status.h"
#include "app/app_io.pb.h"
#include "dataloader.h"
#include "matplotlibcpp17/pyplot.h"
#include "src/google/protobuf/util/time_util.h"
#include "tools/plot/gaussian_error.h"
#include "tools/plot/pivot_table.h"

namespace dyn_delta_approx {
namespace tools {
namespace plot {

namespace {
using dyn_delta_approx::app::app_io::FailedExperiment;
using dyn_delta_approx::app::app_io::Result;
using dyn_delta_approx::app::app_io::Visualisation;
using dyn_delta_approx::app::app_io::VisualisationsFile;
}  // namespace
template <typename T>
struct GroupByEvaluator {
  std::function<T(T)> unary_operator;
  std::function<T(T, T)> binary_operator;
  std::function<T(T, size_t)> final;
  T start_value = T{};
  GroupByEvaluator() {}
  GroupByEvaluator(std::function<T(T)> u, std::function<T(T, T)> b, std::function<T(T, size_t)> f,
                   T s = T{})
      : unary_operator(u), binary_operator(b), final(f), start_value(s) {}
  GroupByEvaluator(const GroupByEvaluator&) = default;
  GroupByEvaluator(GroupByEvaluator&&) = default;
  GroupByEvaluator& operator=(const GroupByEvaluator&) = default;
  GroupByEvaluator& operator=(GroupByEvaluator&&) = default;
};
template <typename T>
GroupByEvaluator<T> MaxEvaluator() {
  return GroupByEvaluator<T>([](T a) { return a; }, [](T a, T b) { return std::max(a, b); },
                             [](T a, size_t) { return a; }, T((int64_t)0));
}
template <typename T>
auto MinEvaluator() {
  return GroupByEvaluator<T>([](T a) { return a; }, [](T a, T b) { return a < b ? a : b; },
                             [](T a, size_t) { return a; }, T(std::numeric_limits<int64_t>::max()));
}
template <typename T>
auto FailEvaluator() {
  return GroupByEvaluator<T>([](T a) { return a; }, [](T a, T b) { return b; },
                             [](T a, size_t n) { return a; }, T{});
};
template <typename T>
auto AvgEvaluator() {  // TODO() Add support for stddev
  return GroupByEvaluator<T>([](T a) { return a; }, [](T a, T b) { return a + b; },
                             [](T a, size_t n) { return a / ((double)n); }, T(0.0f));
};
template <typename T>
auto GeomeanLogEvaluator() {
  return GroupByEvaluator<T>(
      [](T a) -> T { return log(a.to_double()); }, [](T a, T b) { return a + b; },
      [](T a, size_t n) { return exp(a.to_double() / ((double)n)); }, T(0.0f));
};
template <typename D, typename GroupType>
class GroupBy {
  using Predicate = std::function<GroupType(D)>;
  using HavingPredicate = std::function<bool(GroupType, const std::vector<D>&)>;
  using WherePredicate = std::function<bool(const D&)>;

  template <typename A, typename B>
  using MapType = std::map<A, B>;
  MapType<GroupType, std::vector<D>> by_group;
  GroupBy(const MapType<GroupType, std::vector<D>>& dat) : by_group(dat) {}

 public:
  GroupBy(const std::vector<D>& data, Predicate by) {
    for (auto& d : data) {
      by_group[by(d)].push_back(d);
    }
  }
  template <class C>
  GroupBy(const C& container, Predicate by) {
    for (auto elem : container) {
      by_group[by(elem)].push_back(elem);
    }
  }
  size_t max_count() {
    return std::max_element(by_group.begin(), by_group.end(),
                            [](auto a, auto b) { return a.second.size() < b.second.size(); })
        ->second.size();
  }
  MapType<GroupType, size_t> count() {
    MapType<GroupType, size_t> result;
    for (const auto& [k, v] : by_group) {
      result[k] = v.size();
    }
    return result;
  }
  template <class C>
  std::vector<C> transform(GroupType t, std::function<C(D)> op) {
    std::vector<C> result;
    std::transform(by_group[t].begin(), by_group[t].end(), std::back_inserter(result), op);
    return result;
  }
  template <class C>
  std::map<GroupType, std::vector<C>> transform_all(std::function<C(D)> op) {
    std::map<GroupType, std::vector<C>> result;
    for (auto k : keys()) {
      result[k] = transform(k, op);
    }
    return result;
  }

  size_t min_count() {
    return std::min_element(by_group.begin(), by_group.end(),
                            [](auto a, auto b) { return a.second.size() < b.second.size(); })
        ->second.size();
  }
  GroupBy having(HavingPredicate pred) {
    MapType<GroupType, std::vector<D>> data;
    for (const auto& [k, v] : by_group) {
      if (pred(k, v)) {
        std::copy(v.begin(), v.end(), std::back_inserter(data[k]));
      }
    }
    return GroupBy(data);
  }
  GroupBy where(WherePredicate pred) {
    MapType<GroupType, std::vector<D>> data;
    for (const auto& [k, v] : by_group) {
      std::copy_if(v.begin(), v.end(), std::back_inserter(data[k]), pred);
      // std::cout << k << ": " << data[k].size() << std::endl;
    }
    return GroupBy(data);
  }
  template <typename T>
  MapType<GroupType, T> agg(
      std::function<T(D)> access, std::function<T(T, T)> func,
      std::function<T(T, size_t)> final = [](T a) { return a; }, T start_value = T{}) {
    MapType<GroupType, T> result;
    for (auto& [gr, data_points] : by_group) {
      result[gr] = final(
          std::transform_reduce(data_points.begin(), data_points.end(), start_value, func, access),
          data_points.size());
    }
    return result;
  }

  template <typename T>
  MapType<std::array<GroupType, 2>, T> agg_many(
      const std::set<GroupType>& elements, std::function<T(D, const GroupType&)> access,
      MapType<GroupType, GroupByEvaluator<T>> evaluators) {
    MapType<std::array<GroupType, 2>, T> result;
    std::cout << "size: " << elements.size() << std::endl;
    for (const auto& elem : elements) {
      std::function<T(D)> acc = [&](D d) -> T {
        return evaluators.at(elem).unary_operator(access(d, elem));
      };
      auto elems = agg(acc, evaluators.at(elem).binary_operator, evaluators.at(elem).final,
                       evaluators.at(elem).start_value);
      for (auto [k, v] : elem) {
        std::cout << "elem " << k << " " << v.to_string() << std::endl;
      }
      for (auto [k, v] : elems) {
        result[{k, elem}] = v;
      }
    }
    return result;
  }
  template <typename T = double>
  MapType<GroupType, T> geo_mean_log(std::function<T(D)> access) {
    return agg<T>(
        [&](D a) -> T {
          if (access(a) <= 0.0) {
            std::cout << "WARN zero " << access(a) << std::endl;
          }
          if (!std::isfinite(access(a))) {
            std::cout << "NAN " << access(a) << std::endl;
          }
          return log(access(a));
        },
        [](T a, T b) {
          if (!std::isfinite(a)) {
            std::cout << "aNAN " << a << std::endl;
          }
          if (!std::isfinite(b)) {
            std::cout << "bNAN " << b << std::endl;
          }
          return a + b;
        },
        [](T a, size_t n) {
          std::cout << a << " " << n << std::endl;
          return exp(a / ((T)n));
        },
        0);
  }
  template <typename T = double>
  MapType<GroupType, GaussianError<T>> avg(std::function<T(D)> access) {
    auto avg_ =
        agg<T>(access, [](T a, T b) { return a + b; }, [](T a, size_t n) { return a / ((T)n); });

    MapType<GroupType, GaussianError<T>> result;
    for (auto& [gr, data_points] : by_group) {
      std::function<T(D)> curried_avg = [&](auto a) {
        return (access(a) - avg_[gr]) * (access(a) - avg_[gr]);
      };
      result[gr] = GaussianError<T>(
          avg_[gr], std::sqrt(std::transform_reduce(
                                  data_points.begin(), data_points.end(), T{0},
                                  [](auto a, auto b) { return a + b; }, curried_avg) /
                              data_points.size()));
    }
    return result;
  }
  auto keys() {
    std::set<GroupType> result;
    for (const auto& [k, v] : by_group) {
      result.insert(k);
    }
    return result;
  }
  template <typename T = double>
  MapType<GroupType, T> min(std::function<T(D)> access) {
    return agg<T>(
        access, [](T a, T b) { return std::min(a, b); }, [](T a, size_t n) { return a; },
        std::numeric_limits<T>::max());
  }
  template <typename T = double>
  MapType<GroupType, T> max(std::function<T(D)> access) {
    return agg<T>(
        access, [](T a, T b) { return std::max(a, b); }, [](T a, size_t n) { return a; }, T{0});
  }
  auto different_element_count() const { return by_group.size(); }
};

template <typename D, typename GroupType>
class FilterBy {
  using Predicate = std::function<GroupType(D)>;
  template <typename A, typename B>
  using MapType = std::map<A, B>;
  MapType<GroupType, std::vector<D>> by_group;

 public:
  template <class Container>
  FilterBy(const Container& data, Predicate by) {
    for (auto& d : data) {
      by_group[by(d)].push_back(d);
    }
  }
  std::vector<D> at_least_one(std::function<bool(D)> pred) const {
    std::vector<D> result;
    for (const auto& [gr, gr_data] : by_group) {
      if (std::any_of(gr_data.begin(), gr_data.end(), pred)) {
        std::copy(gr_data.begin(), gr_data.end(), std::back_inserter(result));
      }
    }
    return result;
  }
  std::vector<D> having(std::function<bool(GroupType)> pred) const {
    std::vector<D> result;
    for (const auto& [prop, data] : by_group) {
      if (pred(prop)) {
        std::copy(data.begin(), data.end(), std::back_inserter(result));
      }
    }
    return result;
  }
  std::vector<D> all(std::function<bool(D)> pred) const {
    std::vector<D> result;
    for (const auto& [gr, gr_data] : by_group) {
      if (std::all_of(gr_data.begin(), gr_data.end(), pred)) {
        std::copy(gr_data.begin(), gr_data.end(), std::back_inserter(result));
      }
    }
    return result;
  }
};

class VisualisationTool {
  pybind11::scoped_interpreter guard{};
  pybind11::module_ mod, seaborn, pandas, skl, np;
  matplotlibcpp17::pyplot::PyPlot plt;
  VisualisationsFile file;
  void ensurePathsExists(std::string, Visualisation& vis);
  absl::Status plot_performance_profile_internal(Visualisation vis, std::vector<Result>& data,
                                                 std::string sort = "all", int cap = 0);
  std::map<std::string, std::vector<Result>> transform_by_sort(std::vector<ExperimentResult>& data,
                                                               Visualisation vis);
  std::map<std::string, std::vector<FailedExperiment>> transform_by_sort_failed(
      std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status scatter_plot_internal(Visualisation vis, std::vector<Result>& data,
                                     std::string sort = "all", int cap = 0);
  absl::Status table_internal(Visualisation vis, std::vector<Result>& data,
                              std::vector<FailedExperiment>& failed,
                              const std::set<std::string> index, const std::set<std::string> pivot,
                              std::string sort = "all", int64_t cap = 0);
  absl::Status stats_table_internal(Visualisation vis, std::vector<Result>& data,
                                    std::vector<FailedExperiment>& failed,
                                    const std::set<std::string> index,
                                    const std::set<std::string> pivot, std::string sort = "all",
                                    int64_t cap = 0);
  absl::Status stats_from_debug_table_internal(Visualisation vis, std::vector<Result>& data,
                                               std::vector<FailedExperiment>& failed,
                                               std::string sort = "all", int64_t cap = 0);
  absl::Status categorial_bar_plot_internal(Visualisation vis, std::vector<Result>& data,
                                            std::vector<FailedExperiment>& failed,
                                            std::string sort = "all", int cap = 0);
  absl::Status scatter_debug_plot(std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status scatter_debug_plot_internal(Visualisation vis, std::vector<Result>& data,
                                           std::vector<FailedExperiment>& failed,
                                           std::string sort = "all", int cap = 0);
  absl::Status jointplot_plot(std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status jointplot_plot_internal(Visualisation vis, std::vector<Result>& data,
                                       std::vector<FailedExperiment>& failed,
                                       std::string sort = "all", int cap = 0);
  absl::Status categorial_bar_plot(std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status stats_debug(std::vector<ExperimentResult>& data, Visualisation vis);

  absl::Status stats_internal(Visualisation vis, std::vector<Result>& data,
                              std::vector<FailedExperiment>& failed, std::string sort = "all",
                              int cap = 0);
  absl::Status table(std::vector<ExperimentResult>& data, Visualisation vis);

  absl::Status violin_plot_internal(Visualisation vis, std::vector<Result>& data,
                                    std::vector<FailedExperiment>& failed, std::string sort = "all",
                                    int cap = 0);
  absl::Status write_csv_internal(Visualisation vis, std::vector<Result>& data,
                                  std::vector<FailedExperiment>& failed, std::string sort = "all",
                                  int cap = 0);
  absl::Status violin_plot(std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status stats_table(std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status stats(std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status write_csv(std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status efficency(std::vector<ExperimentResult>& data, Visualisation vis);
  absl::Status efficency_internal(matplotlibcpp17::figure::Figure& fig,
                                  matplotlibcpp17::axes::Axes& ax, Visualisation vis,
                                  std::vector<Result>& data, std::string sort = "all", int cap = 0);

  absl::Status plot_internal(
      std::vector<ExperimentResult>& data, Visualisation vis,
      std::function<absl::Status(Visualisation, std::vector<Result>&, std::string, int)> function);
  const std::map<std::string, std::function<Any(Result)>> performance_characteristics = {
      {"first_debug_average_d",
       [](Result r) -> Any {
         return r.algorithm_run_informations()[0].debug_information().double_info().at("average_d");
       }},
      {"first_debug_max_d",
       [](Result r) -> Any {
         return r.algorithm_run_informations()[0].debug_information().int64_info().at("max_d");
       }},
      {"first_debug_m",
       [](Result r) -> Any {
         return r.algorithm_run_informations()[0].debug_information().int64_info().at("m");
       }},
      {"first_debug_n",
       [](Result r) -> Any {
         return r.algorithm_run_informations()[0].debug_information().int64_info().at("n");
       }},
      {"first_debug_max_node_degree",
       [](Result r) -> Any {
         return r.algorithm_run_informations()[0].debug_information().int64_info().at(
             "max_node_degree");
       }},
      {"last_edge_count",
       [](Result r) -> Any {
         return r.algorithm_run_informations()[r.algorithm_run_informations_size() - 1]
             .edge_count();
       }},
      {"runtime",
       [](Result D) -> Any {
         return GaussianError<double>(::google::protobuf::util::TimeUtil::DurationToNanoseconds(
                                          D.run_information().algo_duration()) /
                                      1.0e9);
       }},
      {"runInformation_"
       "algoDuration",
       [](Result D) -> Any {
         return GaussianError<double>(::google::protobuf::util::TimeUtil::DurationToNanoseconds(
                                          D.run_information().algo_duration()) /
                                      1.0e9);
       }},
      {"memory", [](Result D) -> Any { return D.run_information().max_allocated_memory_in_mb(); }},
      {"m/n",
       [](Result D) {
         return ((double)D.hypergraph().edge_count()) / ((double)D.hypergraph().node_count());
       }},
      {"hypergraph_nedge", [](Result D) { return (D.hypergraph().edge_count()); }},
      {"hypergraph_nnode", [](Result D) { return (D.hypergraph().node_count()); }},
      {"size", [](Result D) { return D.size(); }},
      {"weight", [](Result D) { return D.weight(); }},
      {"quality", [](Result D) { return D.quality(); }},

      {"sqrtBound",
       [](Result D) {
         return std::floor((std::sqrt(8.0 * D.hypergraph().edge_count() - 7.0) + 3.0) / 4.0);
       }},
      {"runConfig_"
       "shortName",
       [](Result D) { return D.run_config().short_name(); }},
      {"hypergraph_name_"
       "sort",
       [](Result D) { return D.hypergraph().sort(); }},
      {"hypergraph_name", [](Result D) { return D.hypergraph().name(); }},
      {"hypergraph_name_edge_weight_type",
       [](Result D) { return D.hypergraph().edge_weight_type(); }}};
  const std::map<std::string, std::function<Any(FailedExperiment)>>
      performance_characteristics_failed = {
          {"runtime", [](FailedExperiment D) -> Any { return Any(); }},
          {"runInformation_algoDuration", [](FailedExperiment D) -> Any { return Any(); }},
          {"m/n", [](FailedExperiment D) { return Any(); }},
          {"hypergraph_nedge", [](FailedExperiment D) { return (D.hypergraph().edge_count()); }},
          {"hypergraph_nnode", [](FailedExperiment D) { return (D.hypergraph().node_count()); }},
          {"size", [](FailedExperiment D) { return Any(); }},
          {"weight", [](FailedExperiment D) { return Any(); }},

          {"sqrtBound",
           [](FailedExperiment D) {
             return std::floor((std::sqrt(8.0 * D.hypergraph().edge_count() - 7.0) + 3.0) / 4.0);
           }},
          {"runConfig_shortName", [](FailedExperiment D) { return D.run_config().short_name(); }},
          {"hypergraph_name", [](FailedExperiment D) { return D.hypergraph().name(); }},
          {"hypergraph_name_sort", [](FailedExperiment D) { return D.hypergraph().sort(); }},
          {"hypergraph_name_edge_weight_type",
           [](FailedExperiment D) { return D.hypergraph().edge_weight_type(); }}};
  std::map<std::string, std::pair<std::string, int>> color_line_map;

 public:
  VisualisationTool(const std::string& vis_file);
  absl::Status plot(std::string root_path);
};

}  // namespace plot
}  // namespace tools
};  // namespace dyn_delta_approx