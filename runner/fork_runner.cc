#include <fcntl.h>
#include <google/protobuf/text_format.h>
#include <sched.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "app/algorithms/algorithm_impl.h"
#include "app/app_io.pb.h"
#include "utils/logging.h"
#include "utils/random.h"
#include "utils/systeminfo.h"

#define MSG_TYPE int
#define MSG_SIZE sizeof(int) + 1
std::map<int, int> calls;
#define CHECK_RETURN_VALUE(cal)                                                         \
  calls[__LINE__]++;                                                                    \
  if (cal == -1) {                                                                      \
    std::cout << calls[__LINE__] << "cal " << __FILE__ << ":" << __LINE__ << std::endl; \
    std::cerr << calls[__LINE__] << "cal " << __FILE__ << ":" << __LINE__ << std::endl; \
    for (auto [k, v] : calls) {                                                         \
      std::cout << k << ": " << v << std::endl;                                         \
    }                                                                                   \
  }
ABSL_FLAG(std::string, experiment_path, "",
          "The path to the experiment including definitions file.");
ABSL_FLAG(std::string, root_path, "",
          "Override the root_path stored in the definition file. Should be "
          "used by default!");
ABSL_FLAG(bool, random_order, false, "If experiments should be randomly sorted.");
ABSL_FLAG(bool, dry_run, false, "If experiments should not be executed. Will write results!");
ABSL_FLAG(double, max_alloc_memory, -1,
          "Max memory allocated to this process in total in MB. runner will "
          "only queue as long as this number is not used. Default 100000 MB = 100GB");
ABSL_FLAG(double, max_alloc_memory_per_process, 50000,
          "Max memory allocated to child process in MB. runner will "
          " kill all jobs with higher demand! (default 50000MB)");
ABSL_FLAG(int, cycles_to_queue_new, 30,
          "How many checks have to be run (10 per second) until new job is queued.");
ABSL_FLAG(double, max_idle_memory, 100, "Max Memory consumption of an idle process.");
ABSL_FLAG(int, seed, 0, "Seed for shuffling experiments.");
ABSL_FLAG(double, timeout, -1.0, "Time out for processes (including IO)");
ABSL_FLAG(int, retry_count, 2, "Retry count on Fail (e.g. malloc)");

namespace {
using dyn_delta_approx::app::app_io::ExperimentConfig;
using dyn_delta_approx::app::app_io::ExperimentResultMain;
using dyn_delta_approx::app::app_io::ExperimentResultPart;

using dyn_delta_approx::app::app_io::FailedExperiment;
using dyn_delta_approx::app::app_io::Hypergraph;
using dyn_delta_approx::app::app_io::Result;
using dyn_delta_approx::app::app_io::RunConfig;
using dyn_delta_approx::utils::systeminfo::process_mem_usage;
using namespace std ::chrono_literals;

}  // namespace
   // needed for libtcmalloc and profiler
char* program_invocation_name = "runner";

ExperimentConfig experiment_config;
std::string exp_path;
std::vector<int> fd_request;
std::vector<int> fd_send;
std::vector<pid_t> child_pids;
std::vector<bool> alive;
std::vector<bool> working;
std::vector<double> mem_consumption;
std::vector<double> peak_mem_consumption;
std::vector<std::chrono::steady_clock::time_point> starts;
std::vector<int> currently_working;
std::vector<int> retry_count_field;
std::string root_path_rep;
size_t run_config_size;
size_t initial_number;
bool dry_run = false;
double timeout = -1.0;
int cycles_to_check = 30;
int arg_retry_count = 0;
size_t conc_procs;
double max_memory_per_child = 0;
double max_idle_memory = 100;
int seed = 0;
int finished_jobs = 0;
auto start_time = std::chrono::steady_clock::now();
double max_total_consumption;

std::pair<MSG_TYPE, char> from_buffer(char* buffer) {
  MSG_TYPE res = 0;
  for (int i = MSG_SIZE - 1; i > 1; i--) {
    res |= (unsigned char)buffer[i];
    res = res << 8;
  }
  res |= (unsigned char)buffer[1];
  return {res, buffer[0]};
}
void to_buffer(char data_type, MSG_TYPE msg, char* buffer) {
  buffer[0] = data_type;
  for (int i = 1; i <= MSG_SIZE - 1; i++) {
    buffer[i] = -1 & msg;
    msg = msg >> 8;
  }
}

cpu_set_t mask;

inline void assignToThisCore(int core_id) {
  CPU_ZERO(&mask);
  CPU_SET(core_id, &mask);
  sched_setaffinity(0, sizeof(mask), &mask);
}

void regenerate_worker(int);
int iter = 0;
bool finish_signal = false;
void printStats(int iterator, int tasks) {
  if (++iter % 500 == 0) {
    auto alives = std::accumulate(alive.begin(), alive.end(), 0);
    auto ws = std::accumulate(working.begin(), working.end(), 0);
    double total_mem = std::accumulate(mem_consumption.begin(), mem_consumption.end(), 0.0);
    double duration = std::chrono::duration_cast<std::chrono::duration<double>>(
                          std::chrono::steady_clock::now() - start_time)
                          .count();
    double proj = duration * ((double)tasks) / ((double)finished_jobs);
    std::cout << iterator << " of " << tasks << "/" << initial_number
              << " scheduled. Finished jobs: " << finished_jobs << " dur: " << duration
              << " proj: " << proj << std::endl;
    std::cout << conc_procs << " Procs scheduled. Alive: " << alives << " Working: " << ws
              << " Total mem: " << total_mem / 1000.0 << "GB" << std::endl;
    if (iter % 1500 == 0) {
      std::cout << "Main: " << process_mem_usage() / 1000.0 << std::endl;
      for (int i = 0; i < child_pids.size(); i++) {
        if (currently_working[i] != -1) {
          auto indx_work = currently_working[i] / experiment_config.repetitions();
          std::cout
              << "  " << i << " PID:" << child_pids[i] << " Job: " << currently_working[i] << "("
              << experiment_config.hypergraphs()[indx_work / run_config_size].name() << "["
              << experiment_config.run_configs()[indx_work % run_config_size].short_name() << "])"
              << " Mem: " << mem_consumption[i] / 1000.0 << " GB Time:"
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - starts[i]).count()
              << std::endl;
        }
      }
      iter = 0;
    }
    // print statistics
  }
}
void handle_main_process(bool random_order_enabled) {
  std::atomic<int> iterator = 0;
  std::atomic<int> running_processes = 0;
  initial_number = experiment_config.hypergraphs_size() * experiment_config.run_configs_size() *
                   experiment_config.repetitions();
  std::vector<size_t> id_order(initial_number);
  retry_count_field.resize(initial_number, 0);
  std::iota(id_order.begin(), id_order.end(), 0);
  if (random_order_enabled) {
    std::cout << "Shuffling " << id_order.size() << " elements." << std::endl;
    if (seed == 0) {
      srand(time(NULL));
      seed = rand() % 10000;
    }
    std::cout << "Using seed " << seed << std::endl;
    dyn_delta_approx::utils::Randomize::instance().setSeed(seed);
    dyn_delta_approx::utils::Randomize::instance().shuffleVector(id_order, id_order.size());
    for (auto id : id_order) {
      std::cout << id << " ";
    }
    std::cout << std::endl;
  }

  std::set<MSG_TYPE> already_processed;
  ExperimentResultMain main;
  std::stringstream file_name_stream;
  file_name_stream << exp_path << "/result-main.binary_proto";
  std::ifstream istream(file_name_stream.str());
  if (istream.good()) {
    main.ParseFromIstream(&istream);
    std::cout << "Will skip " << main.id_of_finished_exp_size() << " experiments." << std::endl;
  }
  for (auto id : main.id_of_finished_exp()) {
    already_processed.insert(id);
  }
  int j = 0;
  char buf[MSG_SIZE];
  int nread = 0;
  int checks_since_scheduled = 100;
  while (true) {
    // check on processes
    for (int i = 0; i < child_pids.size(); i++) {
      if (!alive[i] || !working[i]) {  // skip dead and not working
        continue;
      }
      nread = read(fd_request[i], buf, MSG_SIZE);
      bool no_message = false;
      bool end_of_conv = false;
      switch (nread) {
        case -1:
          no_message = true;
          break;
        case 0:
          // end of conversation
          end_of_conv = true;
      }
      if (no_message) {
        // check on memory.
        mem_consumption[i] = process_mem_usage(child_pids[i]);
        peak_mem_consumption[i] = std::max(peak_mem_consumption[i], mem_consumption[i]);
        if (mem_consumption[i] > max_memory_per_child) {
          std::cout << "Process " << i << " with PID: " << child_pids[i] << " consumes "
                    << mem_consumption[i] << std::endl;
          {
            FailedExperiment fe;
            fe.set_reason(::dyn_delta_approx::app::app_io::FailReason::Memory);
            fe.set_memory(mem_consumption[i]);
            auto indx_work = currently_working[i] / experiment_config.repetitions();
            *fe.mutable_hypergraph() = experiment_config.hypergraphs()[indx_work / run_config_size];
            *fe.mutable_run_config() = experiment_config.run_configs()[indx_work % run_config_size];
            *main.mutable_id_of_finished_exp()->Add() = currently_working[i];
            *main.mutable_failed_experiments()->Add() = fe;
            std::ofstream output(file_name_stream.str());
            main.SerializeToOstream(&output);
          }
          kill(child_pids[i], SIGKILL);
          wait(NULL);
          regenerate_worker(i);  //
          std::cout << "TASK: " << currently_working[i] << " failed due to memory." << std::endl;
          // TODO (add more output)
          mem_consumption[i] = process_mem_usage(child_pids[i]);
        } else if (timeout > 0.0 &&
                   std::chrono::duration<double>(std::chrono::steady_clock::now() - starts[i])
                           .count() > timeout) {
          std::cout
              << "Process " << i << " with PID: " << child_pids[i] << " took "
              << std::chrono::duration<double>(std::chrono::steady_clock::now() - starts[i]).count()
              << " seconds. " << std::endl;
          {
            FailedExperiment fe;
            fe.set_reason(::dyn_delta_approx::app::app_io::FailReason::Timeout);
            fe.set_timeout(
                std::chrono::duration<double>(std::chrono::steady_clock::now() - starts[i])
                    .count());
            auto indx_work = currently_working[i] / experiment_config.repetitions();
            for (int s = indx_work * experiment_config.repetitions();
                 s < (indx_work + 1) * experiment_config.repetitions(); s++) {
              already_processed.insert(s);
              *main.mutable_id_of_finished_exp()->Add() = s;
              finished_jobs++;
            }
            *fe.mutable_hypergraph() = experiment_config.hypergraphs()[indx_work / run_config_size];
            *fe.mutable_run_config() = experiment_config.run_configs()[indx_work % run_config_size];
            *main.mutable_id_of_finished_exp()->Add() = currently_working[i];
            *main.mutable_failed_experiments()->Add() = fe;
            std::ofstream output(file_name_stream.str());
            main.SerializeToOstream(&output);
          }
          kill(child_pids[i], SIGKILL);
          wait(NULL);
          regenerate_worker(i);  //
          std::cout << "TASK: " << currently_working[i] << " failed due to timeout." << std::endl;
        }
        continue;
      }
      if (end_of_conv) {
        if (iterator < id_order.size()) {
          std::cout << "[WARN] Process " << i << " has probably segfaulted" << std::endl;
          // TODO setting redo count
          if (retry_count_field[currently_working[i]] < arg_retry_count) {
            retry_count_field[currently_working[i]]++;
            id_order.push_back(currently_working[i]);
          }
        }
        std::cout << "Process " << i << " finished running." << std::endl;
        alive[i] = false;
        mem_consumption[i] = 0;
        peak_mem_consumption[i] = 0;

        int is_alive = kill(child_pids[i], SIGKILL);  // help to die
        if (is_alive > 0) {
          wait(NULL);
        }
        if (iterator < id_order.size()) {
          regenerate_worker(i);
        }
        continue;
      }
      auto returned = from_buffer(buf);
      if (returned.second == 8) {
        float mem = peak_mem_consumption[i];
        to_buffer(8, *((int*)&mem), buf);
        CHECK_RETURN_VALUE(write(fd_send[i], buf, MSG_SIZE));
      } else if (returned.second == 0) {
        if (returned.first != currently_working[i]) {
          if (returned.first < 0) {
            std::cout << "Process " << i << " failed on " << currently_working[i] << std::endl;
          } else {
            std::cout << "Process " << i << " did not return his work " << currently_working[i]
                      << " != " << returned.first << std::endl;
          }
        } else {
          std::cout << "Process " << i << " has finished working on " << currently_working[i]
                    << std::endl;
          finished_jobs++;
          // send new later
          mem_consumption[i] = process_mem_usage(child_pids[i]);
          working[i] = false;
          already_processed.insert(currently_working[i]);
          {
            *main.mutable_id_of_finished_exp()->Add() = currently_working[i];
            std::ofstream output(file_name_stream.str());
            main.SerializeToOstream(&output);
          }
          currently_working[i] = -1;
          if (true || mem_consumption[i] > max_idle_memory) {
            // kill process at random & put the job in the back again.
            std::cout << "Killing Process " << i << " due to idle memory. Used "
                      << mem_consumption[i] << std::endl;
            kill(child_pids[i], SIGKILL);
            wait(NULL);
            regenerate_worker(i);
          }
          checks_since_scheduled = 100;  // quicker find work.
        }
      }
    }
    // print total
    printStats(iterator, id_order.size());
    for (int i = 0; i < child_pids.size(); i++) {
      if (working[i]) {
        mem_consumption[i] = process_mem_usage(child_pids[i]);
        peak_mem_consumption[i] = std::max(peak_mem_consumption[i], mem_consumption[i]);
      } else {
        mem_consumption[i] = 0;
        peak_mem_consumption[i] = 0;
      }
    }
    // check total memory
    int working_procs = std::accumulate(working.begin(), working.end(), 0);
    double total_mem = std::accumulate(mem_consumption.begin(), mem_consumption.end(), 0.0);
    if (total_mem > max_total_consumption) {
      for (int i = 0; i < child_pids.size(); i++) {
        if (working[i] && alive[i]) {
          std::cout << "Process " << i << " with " << child_pids[i] << " consumes "
                    << mem_consumption[i] << " but total memory " << total_mem
                    << " calls to kill it!" << std::endl;
          // kill process at random & put the job in the back again.
          kill(child_pids[i], SIGKILL);
          wait(NULL);
          regenerate_worker(i);  //
          std::cout << "TASK: " << currently_working[i]
                    << " killed due to TOTAL memory. Job pushed back." << std::endl;
          checks_since_scheduled = -100;  // wait 10 secs to schedule jobs
          id_order.push_back(currently_working[i]);
          break;
        }
      }
    } else {
      checks_since_scheduled++;
      if (checks_since_scheduled >
          cycles_to_check) {  // wait to schedule jobs, //TODO: predict memory
        if (!finish_signal && working_procs != working.size()) {
          std::cout << "Could send new job." << std::endl;
        }
        checks_since_scheduled = 0;
        auto projected_mem = (total_mem / working_procs) * (working_procs + 2);
        if (working_procs != 0 && projected_mem > max_total_consumption) {
          std::cout << "Did not send job. Projected memory is " << projected_mem << " exceeding "
                    << max_total_consumption << std::endl;
          checks_since_scheduled = -100;  // wait longer
        } else {
          for (int i = 0; i < child_pids.size(); i++) {
            if (!working[i] && alive[i]) {
              // send new job
              while (iterator < id_order.size() &&
                     already_processed.find(id_order[iterator]) != already_processed.end()) {
                iterator++;
                finished_jobs++;
              }
              if (iterator < id_order.size()) {
                std::cout << "Sending new job: " << id_order[iterator] << " to " << i << std::endl;
                auto curr_iter = id_order[iterator] / experiment_config.repetitions();
                std::cout
                    << "HGR name: "
                    << experiment_config.hypergraphs()[curr_iter / run_config_size].name() << "["
                    << experiment_config.run_configs()[curr_iter % run_config_size].short_name()
                    << "]" << std::endl;
                currently_working[i] = id_order[iterator];
                peak_mem_consumption[i] = 0;
                starts[i] = std::chrono::steady_clock::now();
                working[i] = true;
                to_buffer(0, id_order[iterator], buf);  // send job signal
                CHECK_RETURN_VALUE(write(fd_send[i], buf, MSG_SIZE));
                do {
                  iterator++;
                  std::cout << "Increase iterator." << std::endl;
                } while (iterator < id_order.size() &&
                         already_processed.find(id_order[iterator]) != already_processed.end());
                break;
              } else {
                // send terminate
                to_buffer(0, -std::numeric_limits<MSG_TYPE>::max(),
                          buf);  // send success signal
                CHECK_RETURN_VALUE(write(fd_send[i], buf, MSG_SIZE));
                alive[i] = false;
              }
            }
          }
        }
      }
      if (iterator >= id_order.size()) {
        // send kill signals.
        if (!finish_signal && checks_since_scheduled % 1000 == 0) {
          std::cout << "Sending finished to all not working processes." << std::endl;
        }
        finish_signal = true;

        for (int i = 0; i < child_pids.size(); i++) {
          if (!working[i] && alive[i]) {
            // send terminate
            to_buffer(0, -std::numeric_limits<MSG_TYPE>::max(),
                      buf);  // send success signal
            CHECK_RETURN_VALUE(write(fd_send[i], buf, MSG_SIZE));
            alive[i] = false;
          }
        }
      } else {
        finish_signal = false;
      }
    }

    std::this_thread::sleep_for(100ms);
    auto alives = std::accumulate(alive.begin(), alive.end(), 0);
    if (alives == 0) {
      std::cout << "ZERO process alive. Exiting loop" << std::endl;
      break;
    }
  }
}
void handle_worker_process(int i, int read_pointer, int write_pointer) {
  // assignToThisCore(2 * i);
  //  TODO read already processed again.
  ExperimentResultPart part, debug_part;
  std::stringstream file_name_stream;
  file_name_stream << exp_path << "/result-" << i << "-" << conc_procs << ".binary_proto";

  std::ifstream istream(file_name_stream.str());
  if (istream.good()) {
    part.ParseFromIstream(&istream);
  }
  part.set_concurrent_processes(conc_procs);
  part.set_this_process(i);
  *part.mutable_experiment_name() = experiment_config.experiment_name();
  int nread = -1;
  char buf[MSG_SIZE];
  int iterator;
  std::cout << file_name_stream.str() << std::endl;
  while (true) {
    Hypergraph current;
    RunConfig run_config;
    int curr_iter = 0;

    nread = read(read_pointer, buf, MSG_SIZE);
    bool no_message = false;
    bool end_of_conv = false;

    switch (nread) {
      case -1:
        no_message = true;
        break;
      case 0:
        // end of conversation
        end_of_conv = true;
    }
    if (no_message) {
      std::this_thread::sleep_for(1000ms);
      continue;
    }
    if (end_of_conv) {
      break;
    }
    // get iterator
    // iterator++;
    // TODO: readin
    auto message = from_buffer(buf);
    iterator = message.first;
    if (iterator < 0) {
      exit(0);  // got terminate
    }
    // std::stringstream debugfile_name_stream;
    // debugfile_name_stream << exp_path << "/debug-" << iterator << ".binary_proto";
    // run experiments at the same time
    curr_iter = iterator / experiment_config.repetitions();
    current = experiment_config.hypergraphs()[curr_iter / run_config_size];
    run_config = experiment_config.run_configs()[curr_iter % run_config_size];

    // Do some work
    // adjust file path
    *current.mutable_file_path() =
        absl::StrCat(experiment_config.root_path(), "/", current.file_path());
    std::vector<Result> results;
    bool result_ok = true;
    if (!dry_run) {
      if (!run_config.is_external()) {
        if (run_config.seeds_size() != 0) {
          for (size_t s = 0; s < run_config.seeds_size(); s++) {
            auto curr_seed = run_config.seeds()[s];
            std::srand(curr_seed);
            auto result_s = dyn_delta_approx::app::algorithms::Run(current, run_config, false);
            if (!result_s.ok()) {
              std::cout << "FAILED: " << result_s.status().message() << std::endl;
              result_ok = false;
              break;
            }
            auto result = std::move(result_s.value());
            result.set_seed(curr_seed);
            results.push_back(result);
          }
        } else {
          auto result_s = dyn_delta_approx::app::algorithms::Run(current, run_config, false);
          if (!result_s.ok()) {
            std::cout << "FAILED: " << result_s.status().message() << std::endl;
            result_ok = false;
          }
          results.push_back(result_s.value());
        }
      }
    } else {
      std::this_thread::sleep_for(1s);
      results.push_back(Result());
    }
    // save result or communicate back unsuccessful (negative)
    if (!result_ok) {
      if (iterator == 0) {                                // due to -1
        iterator = std::numeric_limits<MSG_TYPE>::max();  // TODO check this
      }
      to_buffer(0, -iterator, buf);
      CHECK_RETURN_VALUE(write(write_pointer, buf, MSG_SIZE));
    } else {
      std::cout << "SENDING success" << std::endl;
      to_buffer(8, 0, buf);
      CHECK_RETURN_VALUE(write(write_pointer, buf, MSG_SIZE));
      while (read(read_pointer, buf, MSG_SIZE) != MSG_SIZE) {
        std::this_thread::sleep_for(10ms);
      }
      auto message = from_buffer(buf);
      float mem = *((float*)&message.first);
      for (auto val : results) {
        val.mutable_run_information()->set_max_allocated_memory_in_mb(mem);
        *part.mutable_results()->Add() = val;
        *debug_part.mutable_results()->Add() = val;
      }
      // saving part;
      std::ofstream output(file_name_stream.str());
      // std::ofstream debug_output(debugfile_name_stream.str());
      if (!output.is_open()) {
        std::cout << "not opeenenrfereesrsfsefresf" << std::endl;
      }
      if (!part.SerializeToOstream(&output)) {
        std::cout << "ERROR serializing" << std::endl;
      }
      // debug_output.close();
      output.close();
      to_buffer(0, iterator, buf);  // send success signal
      CHECK_RETURN_VALUE(write(write_pointer, buf, MSG_SIZE));
    }
  }
  // saving part;
  part.set_concurrent_processes(conc_procs);
  part.set_this_process(i);
  *part.mutable_experiment_name() = experiment_config.experiment_name();
  std::ofstream output(file_name_stream.str());
  part.SerializeToOstream(&output);
  close(read_pointer);
  close(write_pointer);
}
void regenerate_worker(int i) {
  int req_pip[2];
  int send_pip[2];
  CHECK_RETURN_VALUE(pipe(req_pip));
  fcntl(req_pip[0], F_SETFL, O_NONBLOCK);
  CHECK_RETURN_VALUE(pipe(send_pip));
  fcntl(send_pip[0], F_SETFL, O_NONBLOCK);
  auto pid = fork();
  if (pid == 0) {
    int read_pointer = send_pip[0];
    close(send_pip[1]);
    close(req_pip[0]);

    int write_pointer = req_pip[1];

    handle_worker_process(i, read_pointer, write_pointer);
    exit(0);
  } else {
    // other way around
    close(send_pip[0]);
    close(req_pip[1]);
    if (fd_request[i] != -1) {
      close(fd_request[i]);
    }
    if (fd_send[i] != -1) {
      close(fd_send[i]);
    }
    fd_request[i] = req_pip[0];
    fd_send[i] = send_pip[1];

    mem_consumption[i] = 0;
    alive[i] = true;
    working[i] = false;
    child_pids[i] = pid;
  }
}
int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  exp_path = absl::GetFlag(FLAGS_experiment_path);
  bool random_order_enabled = absl::GetFlag(FLAGS_random_order);
  max_memory_per_child = absl::GetFlag(FLAGS_max_alloc_memory_per_process);
  max_total_consumption = absl::GetFlag(FLAGS_max_alloc_memory);
  dry_run = absl::GetFlag(FLAGS_dry_run);
  cycles_to_check = absl::GetFlag(FLAGS_cycles_to_queue_new);
  max_idle_memory = absl::GetFlag(FLAGS_max_idle_memory);
  seed = absl::GetFlag(FLAGS_seed);
  root_path_rep = absl::GetFlag(FLAGS_root_path);
  timeout = absl::GetFlag(FLAGS_timeout);
  arg_retry_count = absl::GetFlag(FLAGS_retry_count);
  std::cout << "Timeout " << timeout << std::endl
            << "Memory per child " << max_memory_per_child << " MB" << std::endl
            << "Total memory " << max_total_consumption << std::endl;
  if (dry_run) {
    std::cout << "Dry running!" << std::endl
              << "Will not execute experiments, but write files." << std::endl;
  }
  if (max_memory_per_child > max_total_consumption) {
    std::cerr << "Please specify more memory for total_consuption than per child." << std::endl;
    exit(1);
  }
  if (exp_path == "") {
    std::cerr << "Please specify a path." << std::endl;
    return 1;
  }
  std::cout << "Path: " << exp_path << std::endl;
  std::ifstream exp_file(exp_path + "/experiment.textproto");
  if (!exp_file.good()) {
    std::cerr << "The 'experiment.textproto' must exist." << std::endl;
    return 1;
  }
  std::stringstream buffer;
  buffer << exp_file.rdbuf();

  if (!google::protobuf::TextFormat::ParseFromString(buffer.str(), &experiment_config)) {
    std::cerr << "Error parsing 'experiment.textproto'." << std::endl;
    return 1;
  }
  std::cout << " Starting experiment with name: '" << experiment_config.experiment_name() << "'"
            << std::endl;
  std::cout << experiment_config.hypergraphs_size() << " hypergraphs in this experiment."
            << std::endl;
  std::cout << experiment_config.run_configs_size() << " run_configs in this experiment."
            << std::endl;
  if (root_path_rep != "") {
    std::cout << "Replacing root path! New value: " << root_path_rep << std::endl;
    *experiment_config.mutable_root_path() = root_path_rep;
  }

  std::mutex io_output, exp_config_access, array_access;

  int last_commit = 0;
  run_config_size = experiment_config.run_configs_size();
  conc_procs = experiment_config.concurrent_processes();

  fd_send.resize(conc_procs, -1);
  fd_request.resize(conc_procs, -1);
  alive.resize(conc_procs);
  mem_consumption.resize(conc_procs);
  peak_mem_consumption.resize(conc_procs);
  starts.resize(conc_procs);
  working.resize(conc_procs);
  child_pids.resize(conc_procs);

  currently_working.resize(conc_procs);
  std::fill(currently_working.begin(), currently_working.end(), -1);
  for (int i = 0; i < conc_procs; i++) {
    regenerate_worker(i);
  }
  handle_main_process(random_order_enabled);
}
