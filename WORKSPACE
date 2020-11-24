workspace(name = "planner_rules_mcts")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

load("//util:deps.bzl", "planner_rules_mcts_dependencies")
planner_rules_mcts_dependencies()

load("@lexmamcts_project//util:deps.bzl", "lexmamcts_dependencies")
lexmamcts_dependencies()

load("@rule_monitor_project//util:deps.bzl", "rule_monitor_dependencies")
rule_monitor_dependencies()

load("@bark_project//tools:deps.bzl", "bark_dependencies")
bark_dependencies()

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()