#pragma once
#include <memory>

#include "app/algorithms/algorithm_impl.h"

namespace dyn_delta_approx::app::algorithms {
/*
 * StreamableInputType has good():bool method to return whether there is a next stream element
 * StreamableInputType has next():StreamMember returning the next element in Stream
 * OutputType stores the intermediate Results.
 * Output Type should conform to the general Result interface
 * Following methods need to be implemented:
 *   - std::unique_ptr<OutputType> Init(const app_io::AlgorithmConfig& config,const
 * StreamableInputType& stream) Creates the solution object (might use const properties of stream)
 *   - void Stream(const StreamableInputType& element, OutputType& solution)
 *     Streams the element, might apply changes to OutputType
 *   - absl::StatusOr<std::unique_ptr<OutputType>> Transform(OutputType& solution)
 *     Transforms the solution into the result (OutputType)
 */

template <class StreamableInputType, class OutputType, int sample_rate = 100000>
class DynamicAlgorithmImpl : public AlgorithmImpl<StreamableInputType, OutputType> {
  virtual std::unique_ptr<OutputType> Init(const app_io::AlgorithmConfig& config,
                                           const StreamableInputType& stream) = 0;
  virtual std::unique_ptr<OutputType> Init(const app_io::AlgorithmConfig& config,
                                           const StreamableInputType& stream,
                                           app_io::DebugInformation& debug) {
    return Init(config, stream);
  }
  // Stream into solution
  virtual void Stream(const typename StreamableInputType::StreamMember& member,
                      const typename StreamableInputType::Mode mode, OutputType& solution) = 0;
  // Overriden method to stream
  absl::Status Run(const RunConfig& run_config, int index,
                   std::unique_ptr<StreamableInputType> input,
                   std::vector<AlgorithmRunInformation>& results) override {
    TIMED_FUNC(timer2);
    const auto& config = run_config.algorithm_configs().at(index);
    AlgorithmRunInformation result;
    int64_t total_duration = 0;
    auto t1_sys = std::chrono::system_clock::now();
    std::unique_ptr<OutputType> solution =
        Init(config, *input, *result.mutable_debug_information());
    bool is_exact = false;
    int64_t k = 0;
    while (input->good()) {
      const auto [next, mode] = input->next();
      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
      Stream(next, mode, *solution);
      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
      auto dur = t2 - t1;
      total_duration += std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(dur).count();
      if (k % sample_rate == 0) {
        TIMED_SCOPE(timer, std::to_string(k) + "sampling");
        auto* sample = result.add_dynamic_samples();
        sample->set_timestamp(k);
        sample->set_quality(solution->quality());
        sample->set_size(solution->size());
        sample->set_weight(solution->weight());
      }
      k++;
      // TODO capture result every i-th iteration
    }

    auto t2_sys = std::chrono::system_clock::now();
    result.set_is_exact(is_exact);

    result.set_weight(solution->weight());
    result.set_quality(solution->quality());
    result.set_size(solution->size());
    result.set_edge_count(solution->instance().currentNumEdges());
    result.set_node_count(solution->instance().currentNumNodes());
    result.set_free_edges(solution->free_edges_size());
    if (!solution->valid()) {
      return absl::InternalError("Not a valid");
    }
    *result.mutable_start_time() = google::protobuf::util::TimeUtil::TimeTToTimestamp(
        std::chrono::system_clock::to_time_t(t1_sys));
    *result.mutable_end_time() = google::protobuf::util::TimeUtil::TimeTToTimestamp(
        std::chrono::system_clock::to_time_t(t2_sys));
    *result.mutable_algo_duration() =
        google::protobuf::util::TimeUtil::NanosecondsToDuration(total_duration);

    *result.mutable_algorithm_config() = config;
    results.push_back(result);
    if (index + 1 < run_config.algorithm_configs_size()) {
      return AlgorithmImplFactory<OutputType>::getInstance().Run(run_config, index + 1,
                                                                 std::move(solution), results);
    }
    return absl::OkStatus();
  }
};
}  // namespace dyn_delta_approx::app::algorithms