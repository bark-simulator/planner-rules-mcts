// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/behavior/motion_primitives/continuous_actions.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/evaluation/evaluator_collision_agents.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/evaluator_goal_reached.hpp"
#include "bark/world/evaluation/ltl/label_functions/generic_ego_label_function.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits.hpp"
#include "bark/world/tests/make_test_world.hpp"
#include "gtest/gtest.h"
#include "ltl/rule_monitor.h"
#include "mvmcts/mvmcts.h"
#include "src/behavior_rules_mcts.hpp"
#include "src/rules_mcts_state.hpp"
#include "src/util.hpp"

using namespace bark::models::behavior;
using namespace bark::models::dynamic;
using namespace bark::geometry;
using namespace mvmcts;
using bark::commons::ParamsPtr;
using bark::commons::SetterParams;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::models::dynamic::Input;
using bark::models::dynamic::SingleTrackModel;
using bark::models::dynamic::Trajectory;
using bark::world::ObservedWorldPtr;
using bark::world::evaluation::EvaluatorCollisionAgents;
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::evaluation::EvaluatorDrivableArea;
using bark::world::evaluation::EvaluatorGoalReached;
using bark::world::evaluation::GenericEgoLabelFunction;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::prediction::PredictionSettings;
using bark::world::tests::make_test_observed_world;
using bark::world::tests::make_test_world;
using ltl::RuleMonitor;

TEST(multi_agent_test, collision) {
  // Setup prediction models for ego agent and other agents
  ParamsPtr params(new SetterParams());
  params->SetReal("BehaviorRulesMcts::StateParameters::PotentialWeight", 0.0);
  params->SetReal("BehaviorRulesMcts::StateParameters::DesiredVelocityWeight",
                  0.0);
  params->SetBool("BehaviorRulesMcts::MultiAgent", true);
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr ego_prediction_model(
      new BehaviorMPContinuousActions(params));
  double prediction_time_span = 0.3f;
  double velocity_difference = -3.0;
  double rel_distance = 7.0;
  double ego_velocity = 0.0;
  Input u1(2);
  u1 << 0, 0;
  Input u2(2);
  u2 << 50, 1;  //  < crazy action to drive out of the corridors
  Input u3(2);
  u3 << 4.0, 0.0;
  std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
      ->AddMotionPrimitive(u1);
  std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
      ->AddMotionPrimitive(u2);
  std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
      ->AddMotionPrimitive(u3);
  PredictionSettings prediction_settings(ego_prediction_model,
                                         ego_prediction_model);

  // Create an observed world with specific goal definition and the
  // corresponding mcts state
  Polygon polygon(
      Pose(3.5, 3.5, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, 7), Point2d(7, 7),
                           Point2d(7, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(
      std::dynamic_pointer_cast<Polygon>(polygon.Translate(
          Point2d(20, -2.75))));  // < move the goal polygon into the driving
  // corridor in front of the ego vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto observed_world = std::dynamic_pointer_cast<ObservedWorld>(
      make_test_observed_world(2, rel_distance, ego_velocity,
                               velocity_difference, goal_definition_ptr)
          .Clone());
  for (const auto& agent : observed_world->GetAgents()) {
    agent.second->SetDynamicModel(dyn_model);
  }
  State init_state3(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  init_state3 << 0.0, 14.1, -1.75, 0.0, 3.0;
  observed_world->GetAgents().at(3)->SetStateInputHistory(
      {{init_state3, Action(DiscreteAction(0))}});

  observed_world->SetupPrediction(prediction_settings);

  RulesMctsStateParameters state_params(params);
  LabelEvaluators labels;
  labels.emplace_back(
      new GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>("collision_ego"));
  MvmctsState mcts_state(observed_world, {{1, {}}, {2, {}}, {3, {}}},
                         &state_params, {1, 2, 3}, 20, &labels);

  // Initially we should have 3 actions for agents 2 and 3
  EXPECT_EQ(mcts_state.GetNumActions(2), 3);
  EXPECT_EQ(mcts_state.GetNumActions(1), 3);

  std::vector<Reward> rewards;
  // Execute action to cause a collision between agent 2 and 3
  auto mcts_state2 = mcts_state.Execute(JointAction({0, 2, 0}), rewards);

  // Check if collision has occurred
  auto collision_checker = EvaluatorCollisionAgents();
  EXPECT_TRUE(boost::get<bool>(
      collision_checker.Evaluate(*mcts_state2->GetObservedWorld())));

  // After the collision, agents 2 and 3 should only have one action anymore.
  EXPECT_EQ(mcts_state2->GetNumActions(1), 1);
  EXPECT_EQ(mcts_state2->GetNumActions(2), 1);

  auto mcts_state3 = mcts_state2->Execute(JointAction({0, 0, 0}), rewards);
  EXPECT_EQ(mcts_state3->GetNumActions(1), 1);
  EXPECT_EQ(mcts_state3->GetNumActions(2), 1);
  auto world2 = mcts_state2->GetObservedWorld();
  auto world3 = mcts_state3->GetObservedWorld();
  EXPECT_TRUE(world3->GetAgent(2)->GetCurrentPosition() ==
              world2->GetAgent(2)->GetCurrentPosition());
  EXPECT_TRUE(world3->GetAgent(3)->GetCurrentPosition() ==
              world2->GetAgent(3)->GetCurrentPosition());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  //  FLAGS_v = 1;
  return RUN_ALL_TESTS();
}