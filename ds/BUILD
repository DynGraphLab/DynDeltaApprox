load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

config_setting(
    name = "hashing_config",
    values = {"define": "hashing=enabled"},
)

genrule(
    name = "hashing-config",
    srcs = [],
    outs = ["hashing-config.h"],
    cmd = select({
        ":hashing_config": "echo \"#define HASHING_ACTIVATED 1\">>$@",
        "//conditions:default": "echo \"#undef HASHING_ACTIVATED\">>$@",
    }),
    stamp = True,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "empty_problem",
    hdrs = [
        "empty_problem.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//utils:logging",
        "//utils:random_lib",
        "//utils:range",
        "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "indexed_priority_queue",
    hdrs = ["indexed_priority_queue.h"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_library(
    name = "edge_orientation",
    hdrs = ["edge_orientation.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "undirected_graph",
    hdrs = ["undirected_graph.h"],
    visibility = ["//visibility:public"],
    deps = ["//utils:range"],
)

cc_library(
    name = "addressable_priority_queue",
    hdrs = ["addressable_priority_queue.h"],
    visibility = ["//visibility:public"],
    deps = [
    ],
)

cc_test(
    name = "addressable_priority_queue_test",
    srcs = ["addressable_priority_queue_test.cc"],
    deps = [
        ":addressable_priority_queue",
        "//utils:range",
        "@googletest//:gtest_main",
    ],
)
