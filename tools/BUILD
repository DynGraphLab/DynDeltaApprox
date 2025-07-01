load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "binary_to_textproto",
    srcs = ["binary_to_textproto.cc"],
    deps = [
        "//app:app_io_cc_proto",
        "//runner:hypergraph_storage_system",
        "@abseil-cpp//absl/strings",
    ],
)
