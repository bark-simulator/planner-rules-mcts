cc_test(
    name = "single_agent_test",
    srcs = [
        "single_agent_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//src:planner_mvmcts",
        "@bark_project//bark/models/behavior",
        "@bark_project//bark/models/behavior/constant_acceleration",
        "@bark_project//bark/models/behavior/motion_primitives",
        "@bark_project//bark/world",
        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
        "@bark_project//bark/world/evaluation/ltl/label_functions:label_function",
        "@bark_project//bark/world/goal_definition",
        "@bark_project//bark/world/tests:make_test_world",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "multi_agent_test",
    srcs = [
        "multi_agent_test.cpp",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//src:planner_mvmcts",
        "@bark_project//bark/models/behavior",
        "@bark_project//bark/models/behavior/constant_acceleration",
        "@bark_project//bark/models/behavior/motion_primitives",
        "@bark_project//bark/world",
        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//bark/world/evaluation:evaluator_collision_agents",
        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
        "@bark_project//bark/world/evaluation/ltl/label_functions:label_function",
        "@bark_project//bark/world/goal_definition",
        "@bark_project//bark/world/tests:make_test_world",
        "@gtest//:gtest_main",
    ],
)

#cc_test(
#    name = "grid_world_threshold_test",
#    srcs = [
#        "grid_world_threshold_test.cpp",
#    ],
#    copts = ["-Iexternal/gtest/include"],
#    deps = [
#        "//gridworld:grid_world_state",
#        "//gridworld:state_file_writer",
#        "//gridworld:test_runner",
#        "@com_github_google_glog//:glog",
#        "@gtest//:gtest_main",
#        "@momamcts_project//mvmcts",
#    ],
#)
