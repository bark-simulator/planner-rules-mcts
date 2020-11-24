load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

def planner_rules_mcts_dependencies():

    _maybe(
        git_repository,
        name = "bark_project",
        commit = "6d644860b3c6a81fef0e191b8dfef16dfd2ae0c6",
        remote = "https://github.com/bark-simulator/bark",
    )

    # _maybe(
    # native.local_repository,
    # name = "bark_project",
    # path = "/home/esterle/development/bark",
    # )

    _maybe(
        git_repository,
        name = "lexmamcts_project",
        commit = "5808885b7d2882fc1956da98efc43eeb039cd7e2",
        remote = "https://github.com/klemense1/lexmamcts",
    )

    _maybe(
        git_repository,
        name = "rule_monitor_project",
        commit = "d1d7830e53213bc69d77c80ba0cc8397b0971c9d",
        remote = "https://github.com/bark-simulator/rule-monitoring",
    )

    _maybe(
        native.new_local_repository,
        name = "python_linux",
        path = "./python/venv/",
        build_file_content = """
cc_library(
    name = "python-lib",
    srcs = glob(["lib/libpython3.*", "libs/python3.lib", "libs/python36.lib"]),
    hdrs = glob(["include/**/*.h", "include/*.h"]),
    includes = ["include/python3.6m", "include", "include/python3.7m", "include/python3.5m"],
    visibility = ["//visibility:public"],
)
    """,
    )

    _maybe(
        http_archive,
        name = "pybind11",
        strip_prefix = "pybind11-2.3.0",
        urls = ["https://github.com/pybind/pybind11/archive/v2.3.0.zip"],
        build_file_content = """
cc_library(
    name = "pybind11",
    hdrs = glob([
    "include/**/**/*.h",
    ]),
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "include/"
)
    """,
    )

    _maybe(
        git_repository,
        name = "com_github_gflags_gflags",
        commit = "addd749114fab4f24b7ea1e0f2f837584389e52c",
        remote = "https://github.com/gflags/gflags",
    )

    _maybe(
        git_repository,
        name = "com_github_google_glog",
        commit = "3ba8976592274bc1f907c402ce22558011d6fc5e",
        remote = "https://github.com/google/glog",
    )
    _maybe(
    git_repository,
    name = "gtest",
    commit = "703bd9caab50b139428cea1aaff9974ebee5742e",
    remote = "https://github.com/google/googletest"
    )

    _maybe(
        http_archive,
        # Need Eigen 3.4 (which is in development) for STL-compatible iterators
        name = "com_github_eigen_eigen",
        build_file_content = """
cc_library(
    name = 'eigen',
    srcs = [],
    includes = ['.'],
    hdrs = glob(['Eigen/**']),
    visibility = ['//visibility:public'],
)""",
        sha256 = "4b1120abc5d4a63620a886dcc5d7a7a27bf5b048c8c74ac57521dd27845b1d9f",
        strip_prefix = "eigen-git-mirror-98e54de5e25aefc6b984c168fb3009868a93e217",
        urls = [
            "https://github.com/eigenteam/eigen-git-mirror/archive/98e54de5e25aefc6b984c168fb3009868a93e217.zip",
        ],
    )

    _maybe(
        git_repository,
        name = "com_github_nelhage_rules_boost",
        branch = "master",
        remote = "https://github.com/nelhage/rules_boost",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)
