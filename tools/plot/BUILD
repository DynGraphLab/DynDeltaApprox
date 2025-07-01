load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "dataloader",
    srcs = ["dataloader.cc"],
    hdrs = ["dataloader.h"],
    deps = [
        "//app:app_io_cc_proto",
        "@abseil-cpp//absl/status:statusor",
        "@protobuf//src/google/protobuf/util:time_util",
    ],
)

cc_binary(
    name = "plot_cc",
    srcs = [
        "gaussian_error.h",
        "pivot_table.h",
        "plot.cc",
        "plot.h",
    ],
    deps = [
        ":dataloader",
        "@matplotlibcpp17",
        "@protobuf//src/google/protobuf/util:time_util",
        "@spack_python",
    ],
)
