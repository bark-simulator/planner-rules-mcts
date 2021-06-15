load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

def planner_rules_mcts_dependencies():

    _maybe(
        git_repository,
        name = "bark_project",
        commit = "9649bc11b304d42b62293352c8a4052789d553a2",
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
        commit = "24ab42ca3dd67c15d8e86955d5d807b86ad3860d",
        remote = "https://github.com/klemense1/lexmamcts",
    )

    _maybe(
        git_repository,
        name = "rule_monitor_project",
        commit = "187c125a18979214d638ca771dd86e7934932b94",
        remote = "https://github.com/bark-simulator/rule-monitoring",
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
      name = "com_github_glog_glog",
      commit = "c5dcae830670bfaea9573fa7b700e862833d14ff",
      remote = "https://github.com/google/glog"
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
