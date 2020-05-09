load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

def planner_mv_mvts_dependencies():
    # _maybe(
    # git_repository,
    # name = "bark_project",
    # commit="042ed605f5631cfd59cc8dc9bc68325de4412d19",
    # remote = "https://github.com/bark-simulator/bark",
    # )

    _maybe(
    git_repository,
    name = "bark_project",
    commit="50ec3c88aef008982e28d7b6f6d61267c391e9a0",
    remote = "git@github.com:klemense1/mv-mamcts.git",
    )

    # _maybe(
    # native.local_repository,
    # name = "bark_project",
    # path = "/home/esterle/development/bark",
    # )

    _maybe(
    git_repository,
    name = "momamcts_project", # was mcts
    commit = "f918f7b98161efa8f1921aea9e0688688f7b5162",
    remote = "git@github.com:cirrostratus1/momamcts.git",
    )

    _maybe(
    git_repository,
    name = "rule_monitor_project",
    commit = "0a618b3e10755e1feb19f9859943fe619a4f923b",
    remote = "git@github.com:bark-simulator/rule-monitoring.git",
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
    """
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
    """
    )

    _maybe(
    git_repository,
    name = "com_github_gflags_gflags",
    commit = "addd749114fab4f24b7ea1e0f2f837584389e52c",
    remote = "https://github.com/gflags/gflags"
    )

    _maybe(
      git_repository,
      name = "com_github_google_glog",
      commit = "3ba8976592274bc1f907c402ce22558011d6fc5e",
      remote = "https://github.com/google/glog"
    )
    _maybe(
    http_archive,
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file_content = """
cc_library(
    name = "main",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/gtest-all.cc"]
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h"
    ]),
    copts = ["-Iexternal/gtest/include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)""",
    strip_prefix = "googletest-release-1.7.0",
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
    remote = "https://github.com/nelhage/rules_boost"
    )

    # _maybe(
    # http_archive,
    # name = "spot",
    # build_file = "@//tools:spot.BUILD",
    # patch_cmds = ["./configure"],
    # sha256 = "dcb7aa684725304afb3d435f26f25b51fbd6e9a6ef610e16163cc0030ad5eab4",
    # strip_prefix = "spot-2.8.1",
    # urls = ["http://www.lrde.epita.fr/dload/spot/spot-2.8.1.tar.gz"],
    # )


def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)