cc_library(
    name = "util",
    srcs = ["util.cpp"],
    hdrs = ["util.hpp"],
    deps = [
        "@bark_project//modules/commons/params:include",
        "@bark_project//modules/models/behavior:include",
        "@bark_project//modules/world:include",
        "@momamcts_project//mcts",
    ],
)

cc_library(
    name = "mvmcts_state_parameters",
    srcs = ["mvmcts_state_parameters.cpp"],
    hdrs = ["mvmcts_state_parameters.hpp"],
    deps = [
        "@bark_project//modules/commons/params",
        "@bark_project//modules/world:include",
    ],
)

cc_library(
    name = "planner_mvmcts_multi_agent",
    srcs = [
        "mvmcts_state_multi_agent.cpp",
    ],
    hdrs = [
        "behavior_mcts_multi_agent.hpp",
        "mvmcts_state_multi_agent.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":util",
        ":mvmcts_state_parameters",
        "@bark_project//modules:include",
        "@bark_project//modules/models/behavior/motion_primitives:motion_primitives",
        "@com_github_google_glog//:glog",
        "@momamcts_project//ltl:rule_monitor",
        "@momamcts_project//mcts",
    ],
)

cc_library(
    name = "include",
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
        "@momamcts_project//mcts",
    ],
)
