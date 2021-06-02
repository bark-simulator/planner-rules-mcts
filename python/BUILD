package(default_visibility = ["//visibility:public"])
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

cc_library(
    name = "planner_rules_mcts",
    srcs = ["python_planner_rules_mcts.cpp"],
    hdrs = ["python_planner_rules_mcts.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//src:planner_rules_mcts",
        "@bark_project//bark/python_wrapper:common",
        "@com_github_eigen_eigen//:eigen",
        "@com_github_glog_glog//:glog"
    ],
)

# the following rules are only for testing purposes
pybind_extension(
    name = "planner_rules_mcts",
    srcs = ["python_test_module.cpp",
            "python_planner_rules_mcts.cpp"],
    linkstatic = 1,
    deps = [
        ":planner_rules_mcts",
        "@rule_monitor_project//python/bindings:rule_monitor.so",
        "@bark_project//bark/models/behavior",
        "@bark_project//bark/models/behavior/constant_acceleration",
        "@bark_project//bark/models/behavior/motion_primitives",
        "@bark_project//bark/models/behavior/rule_based:lane_change_behavior",
        "@bark_project//bark/world",
        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
        "@bark_project//bark/python_wrapper:behavior",
    ],
)

#py_test(
#    name = "python_import_test",
#    srcs = ["python_import_test.py"],
#    data = [":planner_rules_mcts.so"],
#    imports = ["../python/"],
#)

py_test(
  name = "pickle_unpickle_test",
  srcs = ["pickle_unpickle_test.py"],
  data = [':planner_rules_mcts.so'],
  imports = ['../../python/'],
  deps = ["@bark_project//bark/runtime/commons"]
)