cc_library(
    name = "util",
    srcs = ["util.cpp"],
    hdrs = ["util.hpp"],
    deps = [
        "@bark_project//bark/commons/params:include",
        "@bark_project//bark/models/behavior:include",
        "@bark_project//bark/world:include",
        "@lexmamcts_project//mvmcts",
    ],
)

cc_library(
    name = "rules_mcts_state_parameters",
    srcs = ["rules_mcts_state_parameters.cpp"],
    hdrs = ["rules_mcts_state_parameters.hpp"],
    deps = [
        "@bark_project//bark/commons/params",
        "@bark_project//bark/world:include",
    ],
)

cc_library(
    name = "planner_rules_mcts",
    srcs = [
        "rules_mcts_state.cpp",
    ],
    hdrs = [
        "behavior_rules_mcts.hpp",
        "rules_mcts_state.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":util",
        ":rules_mcts_state_parameters",
        "@bark_project//bark:include",
        "@bark_project//bark/world/evaluation/ltl/label_functions:base_label_function",
        "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
        "@com_github_glog_glog//:glog",
        "@rule_monitor_project//ltl:rule_monitor",
        "@lexmamcts_project//mvmcts",
    ],
)

cc_library(
    name = "include",
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
        "@lexmamcts_project//mvmcts",
    ],
)
