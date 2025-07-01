load("@protobuf//:protobuf.bzl", "py_proto_library")
load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_proto//proto:defs.bzl", "proto_library")

cc_binary(
    name = "app",
    srcs = [
        "app.cc",
        "//:build-info.h",
    ],
    malloc = select({
        "//:gperftools_tcmalloc": "@gperftools",
        "//conditions:default": "@bazel_tools//tools/cpp:malloc",
    }),
    deps = [
        ":app_io_cc_proto",
        "//app/algorithms/dynamic/edge_orientation",
        "//utils:logging",
        "//utils:random_lib",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/flags:parse",
        "@abseil-cpp//absl/status:statusor",
    ],
)

cc_proto_library(
    name = "app_io_cc_proto",
    visibility = ["//visibility:public"],
    deps = [":app_io_proto"],
)

proto_library(
    name = "app_io_proto",
    srcs = ["app_io.proto"],
    deps = [
        "@protobuf//:duration_proto",
        "@protobuf//:timestamp_proto",
    ],
)

py_proto_library(
    name = "app_io_py_proto",
    srcs = ["app_io.proto"],
    visibility = ["//visibility:public"],
)
