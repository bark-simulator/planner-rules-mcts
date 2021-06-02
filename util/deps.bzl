load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

def planner_rules_mcts_dependencies():

    # _maybe(
    #     git_repository,
    #     name = "bark_project",
    #     commit = "580a27b3fc13199d835b15bdda0299e84265fc25",
    #     remote = "https://github.com/bark-simulator/bark",
    # )

    _maybe(
        git_repository,
        name = "bark_project",
        branch = "fix_nightly_build",
        remote = "https://github.com/bark-simulator/bark",
    )

    _maybe(
        git_repository,
        name = "pybind11_bazel",
        commit="c4a29062b77bf42836d995f6ce802f642cffb939",
        remote = "https://github.com/bark-simulator/pybind11_bazel"
    )

    # _maybe(
    # native.local_repository,
    # name = "bark_project",
    # path = "/home/esterle/development/bark",
    # )

    _maybe(
        git_repository,
        name = "lexmamcts_project",
        commit = "540d62c5cfe06aa7313ab2da5b7ea7ae9606ad9c",
        remote = "https://github.com/klemense1/lexmamcts",
    )

    _maybe(
        git_repository,
        name = "rule_monitor_project",
        commit = "73a435b779b13ac54a3f5d22e977161b11a3a96a",
        remote = "https://github.com/bark-simulator/rule-monitoring",
    )

    _maybe(
        git_repository,
        name = "com_github_gflags_gflags",
        commit = "addd749114fab4f24b7ea1e0f2f837584389e52c",
        remote = "https://github.com/gflags/gflags",
    )

    _maybe(
        git_repository,
        name = "com_github_glog_glog",
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

    _maybe(
      new_git_repository,
      name = "com_github_spline",
      commit = "619c634ef5f6f2df1508c767f979eb4b7bf9c66a",
      remote = "https://github.com/ttk592/spline",
      build_file_content = """
cc_library(
    name = 'spline',
    srcs = [],
    includes = ['.'],
    hdrs = ["src/spline.h"],
    visibility = ['//visibility:public'],
)
    """
    )
def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)
