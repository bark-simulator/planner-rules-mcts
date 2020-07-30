cc_test(
    name = "single_agent_test",
    srcs = [
        "single_agent_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@bark_project//bark/models/behavior",
        "@bark_project//bark/models/behavior/constant_velocity",
        "@bark_project//bark/models/behavior/motion_primitives",
        "//src:planner_mvmcts",
        "@bark_project//bark/world/tests:make_test_world",
        "@bark_project//bark/world",
        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
        "@bark_project//bark/world/evaluation/ltl/label_functions:label_function",
        "@bark_project//bark/world/goal_definition",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "grid_world_threshold_test",
    srcs = [
        "grid_world_threshold_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//gridworld:test_runner",
        "//gridworld:grid_world_state",
        "//gridworld:state_file_writer",
        "@com_github_google_glog//:glog",
        "@gtest//:gtest_main",
        "@momamcts_project//mvmcts",
    ],
)
