package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner_mvmcts",
    srcs = ["python_planner_uct.cpp"],
    hdrs = ["python_planner_uct.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//src:planner_mvmcts_multi_agent",
        "@bark_project//python:common",
        "@com_github_eigen_eigen//:eigen",
        "@pybind11",
        "@python_linux//:python-lib",
    ],
)

# the following rules are only for testing purposes
#cc_binary(
#    name = "planner_mvmcts.so",
#    srcs = ["python_test_module.cpp"],
#    linkshared = 1,
#    linkstatic = 1,
#    deps = [
#        ":planner_mvmcts",
#        "@rule_monitor_project//python/bindings:rule_monitor",
#        "@bark_project//modules/models/behavior",
#        "@bark_project//modules/models/behavior/constant_velocity",
#        "@bark_project//modules/models/behavior/motion_primitives",
#        "@bark_project//modules/models/behavior/mobil",
#        "@bark_project//modules/world",
#        "@bark_project//modules/world/evaluation:evaluator_collision_ego_agent",
#        "@bark_project//modules/world/evaluation:evaluator_goal_reached",
#        "@bark_project//python:behavior",
#    ],
#)

#py_test(
#    name = "python_import_test",
#    srcs = ["python_import_test.py"],
#    data = [":planner_mvmcts.so"],
#    imports = ["../python/"],
#)

py_test(
  name = "pickle_unpickle_test",
  srcs = ["pickle_unpickle_test.py"],
  data = [':planner_mvmcts.so'],
  imports = ['../../python/'],
  deps = ["@bark_project//modules/runtime/commons:parameters"]
)