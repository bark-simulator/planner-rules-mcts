cc_library(
    name = "test_runner",
    srcs = ["test_runner.cpp"],
    hdrs = ["test_runner.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//gridworld:grid_world_env",
        "//gridworld:grid_world_state",
        "@boost//:math",
        "@boost//:multiprecision",
        "@com_github_google_glog//:glog",
        "@momamcts_project//evaluation",
        "@momamcts_project//mvmcts",
    ],
)

cc_library(
    name = "grid_world_env",
    srcs = [
        "base_test_env.cpp",
    ],
    hdrs = [
        "base_test_env.h",
        "grid_world_env.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//gridworld:grid_world_state",
        "@com_github_eigen_eigen//:eigen",
        "@com_github_google_glog//:glog",
        "@momamcts_project//mvmcts",
    ],
)

cc_binary(
    name = "threshold_comparison",
    srcs = [
        "threshold_comparison.cpp",
    ],
    deps = [
        ":test_runner",
        "//gridworld:grid_world_state",
        "//gridworld:state_file_writer",
        "@com_github_google_glog//:glog",
        "@momamcts_project//mvmcts",
    ],
)

cc_library(
    name = "common",
    srcs = ["common.cpp"],
    hdrs = ["common.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "@momamcts_project//mvmcts",
    ],
)

cc_library(
    name = "state_file_writer",
    srcs = ["state_file_writer.cpp"],
    hdrs = ["state_file_writer.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
    ],
)

cc_library(
    name = "grid_world_state",
    srcs = [
        "grid_world_state.cpp",
    ],
    hdrs = [
        "grid_world_state.hpp",
        "grid_world_state_parameter.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":common",
        "//gridworld/label_evaluator:label_evaluators",
        "@com_github_google_glog//:glog",
        "@momamcts_project//mvmcts",
        "@rule_monitor_project//ltl:rule_monitor",
    ],
)
