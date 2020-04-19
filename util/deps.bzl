load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def planner_uct_rules_dependencies():
    # _maybe(
    # git_repository,
    # name = "bark_project",
    # commit="042ed605f5631cfd59cc8dc9bc68325de4412d19",
    # remote = "https://github.com/bark-simulator/bark",
    # )

    _maybe(
    git_repository,
    name = "bark_project",
    commit="08d98038312af11101bd868f96337292bbb5bfca",
    remote = "https://github.com/klemense1/mv-mamcts",
    )

    # _maybe(
    # native.local_repository,
    # name = "bark_project",
    # path = "/home/esterle/development/bark",
    # )

    _maybe(
    git_repository,
    name = "momamcts_project", # was mcts
    commit = "deace9238281694b1901a7178e688cfa96a0dbd2",
    remote = "git@github.com:cirrostratus1/momamcts.git",
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