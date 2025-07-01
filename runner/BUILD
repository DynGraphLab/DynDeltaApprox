load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_binary(
    name = "fork_runner",
    srcs = ["fork_runner.cc"],
    linkopts = [],
    malloc = select({
        "//:gperftools_tcmalloc": "@gperftools",
        "//conditions:default": "@bazel_tools//tools/cpp:malloc",
    }),
    deps = [
        "//app:app_io_cc_proto",
        "//app/algorithms/dynamic/edge_orientation",
        "//utils:logging",
        "//utils:random_lib",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/flags:parse",
        "@abseil-cpp//absl/status:statusor",
    ],
)

cc_binary(
    name = "generate_experiment_config",
    srcs = ["generate_experiment_config.cc"],
    deps = [
        ":hypergraph_storage_system",
        "//app:app_io_cc_proto",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/flags:parse",
        "@abseil-cpp//absl/status:statusor",
    ],
)

cc_binary(
    name = "humanize",
    srcs = ["humanize.cc"],
    deps = [
        "//app:app_io_cc_proto",
        "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "hypergraph_storage_system",
    srcs = ["hypergraph_storage_system.cc"],
    hdrs = ["hypergraph_storage_system.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//app:app_io_cc_proto",
        "@abseil-cpp//absl/strings",
    ],
)
