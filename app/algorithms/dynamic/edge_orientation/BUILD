load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "edge_orientation",
    srcs = ["dyn_delta.cc"],
    hdrs = [
        "cchhqrs.h",
        "improved_bfs.h",
        "limited_bfs.h",
        "packed_cchhqrs.h",
        "strong_bfs.h",
    ],
    defines = select({
        ":validate_orientation": ["VALIDATE_EDGE_ORIENTATION"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "//app/algorithms:algorithm_base_impl_lib",
        "//ds/dynamic:graph",
        "//dynamic_edge_orientation/cchhqrs",
    ],
    alwayslink = 1,
)

config_setting(
    name = "validate_orientation",
    values = {"define": "validate_orientation=enabled"},
)
