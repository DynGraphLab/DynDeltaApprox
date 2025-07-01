load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

cc_library(
    name = "binary_reader",
    hdrs = [
        "binary_reader.h",
        "binary_utils.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//utils:logging",
        "@abseil-cpp//absl/status:statusor",
    ],
)

cc_library(
    name = "binary_writer",
    hdrs = [
        "binary_utils.h",
        "binary_writer.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//utils:logging",
        "@abseil-cpp//absl/status:statusor",
    ],
)

cc_library(
    name = "seq_reader",
    hdrs = ["seq_reader.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//utils:logging",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "graph_reader",
    hdrs = ["graph_reader.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//utils:logging",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "mtx_reader",
    hdrs = ["mtx_reader.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//utils:logging",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)
