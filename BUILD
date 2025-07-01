genrule(
    name = "build-info",
    srcs = [],
    outs = ["build-info.h"],
    cmd = select({
        "//:gperftools_tcmalloc": "cat bazel-out/stable-status.txt | while read line; do echo \"#define $$line\" >> $@; done; echo \"#define MALLOC_IMPLEMENTATION \\\"tc-malloc\\\"\">>$@",
        "//conditions:default": "cat bazel-out/stable-status.txt | while read line; do echo \"#define $$line\" >> $@; done; echo \"#define MALLOC_IMPLEMENTATION \\\"default\\\"\">>$@",
    }),
    stamp = True,
    visibility = ["//visibility:public"],
)

config_setting(
    name = "gperftools_tcmalloc",
    values = {"define": "tcmalloc=gperftools"},
)

config_setting(
    name = "spack",
    values = {"define": "spack=enabled"},
)

config_setting(
    name = "logging",
    values = {"define": "logging=enabled"},
)

config_setting(
    name = "kagen",
    values = {"define": "kagen=enabled"},
)

config_setting(
    name = "detailed_logging",
    values = {"define": "detailed_logging=enabled"},
)
