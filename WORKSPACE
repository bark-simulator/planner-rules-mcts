workspace(name = "planner_mv_mvts")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

load("//util:deps.bzl", "planner_mv_mvts_dependencies")
planner_mv_mvts_dependencies()

load("@momamcts_project//util:deps.bzl", "momamcts_dependencies")
momamcts_dependencies()

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()