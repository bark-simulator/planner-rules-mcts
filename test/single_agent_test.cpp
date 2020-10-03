// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/behavior/motion_primitives/continuous_actions.hpp"
#include "bark/models/dynamic/single_track.hpp"
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
#include "src/behavior_mvmcts.hpp"
#include "src/mvmcts_state.hpp"
#include "src/util.hpp"

using namespace bark::models::behavior;
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
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::evaluation::EvaluatorDrivableArea;
using bark::world::evaluation::EvaluatorGoalReached;
using bark::world::evaluation::GenericEgoLabelFunction;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::goal_definition::GoalDefinitionStateLimits;
using bark::world::prediction::PredictionSettings;
using bark::world::tests::make_test_observed_world;
using bark::world::tests::make_test_world;
using ltl::RuleMonitor;

LabelEvaluators make_labels() {
  LabelEvaluators labels;
  labels.emplace_back(
      new GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>("collision_ego"));
  labels.emplace_back(
      new GenericEgoLabelFunction<EvaluatorGoalReached>("goal_reached"));
  labels.emplace_back(
      new GenericEgoLabelFunction<EvaluatorDrivableArea>("collision_corridor"));
  return labels;
}

std::vector<std::shared_ptr<RuleMonitor>> make_rules() {
  std::vector<std::shared_ptr<RuleMonitor>> rules;
  rules.emplace_back(RuleMonitor::MakeRule("G !collision_ego", -1000, 0));
  return rules;
}

MultiAgentRuleState make_rule_states(
    const std::vector<std::shared_ptr<RuleMonitor>>& rules) {
  std::vector<RuleState> rule_states;
  for (const auto& rule : rules) {
    auto rs = rule->MakeRuleState();
    rule_states.insert(rule_states.end(), rs.begin(), rs.end());
  }
  return {{1, rule_states}};
}

TEST(single_agent_mvmcts_state, execute) {
  // Setup prediction models for ego agent and other agents
  ParamsPtr params(new SetterParams());
  params->SetReal("BehaviorMvmcts::StateParameters::PotentialWeight", 0.0);
  params->SetReal("BehaviorMvmcts::StateParameters::DesiredVelocityWeight",
                  0.0);
  params->SetBool("BehaviorMvmcts::MultiAgent", false);
  DynamicModelPtr dyn_model(new SingleTrackModel(params));
  BehaviorModelPtr ego_prediction_model(
      new BehaviorMPContinuousActions(params));
  float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference = 0.0,
        prediction_time_span = 0.3f;
  Input u1(2);
  u1 << 0, 0;
  Input u2(2);
  u2 << 50, 1;  //  < crazy action to drive out of the corridors
  Input u3(2);
  u3 << (rel_distance + 4) * 2 / (prediction_time_span * prediction_time_span),
      0;  //  < action to drive into other agent with a single step
          //   (4m vehicle length assumed)
  std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
      ->AddMotionPrimitive(u1);
  std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
      ->AddMotionPrimitive(u2);
  std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
      ->AddMotionPrimitive(u3);
  BehaviorModelPtr others_prediction_model(
      new BehaviorConstantAcceleration(params));
  PredictionSettings prediction_settings(ego_prediction_model,
                                         others_prediction_model);

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
      make_test_observed_world(1, rel_distance, ego_velocity,
                               velocity_difference, goal_definition_ptr)
          .Clone());
  for (const auto& agent : observed_world->GetAgents()) {
    agent.second->SetDynamicModel(dyn_model);
  }
  observed_world->SetupPrediction(prediction_settings);

  auto label_evaluators = make_labels();
  MvmctsStateParameters state_params(params);
  MvmctsState mcts_state(observed_world, make_rule_states(make_rules()),
                         &state_params, {1}, 20, &label_evaluators);

  std::vector<Reward> rewards;
  auto next_mcts_state = mcts_state.Execute(JointAction({0}), rewards);
  rewards += next_mcts_state->GetTerminalReward();
  // Checking reward of zero if we are neither colliding or at goal
  EXPECT_FALSE(
      next_mcts_state
          ->IsTerminal());  // < make test world is defined in such a way that
                            // a long driving corridor along x exists
                            // no collision should occur after one action
  EXPECT_NEAR(rewards[0](0), 0, 0.00001);

  // Checking collision with corridor
  next_mcts_state = next_mcts_state->Execute(JointAction({1}), rewards);

  EXPECT_TRUE(
      next_mcts_state->IsTerminal());  // < crazy action should lead to a
                                       // collision with driving corridor
  rewards += next_mcts_state->GetTerminalReward();
  EXPECT_NEAR(rewards[0](0), -800, 0.00001);

  // Checking collision with other agent ( use initial state again)
  next_mcts_state = mcts_state.Execute(JointAction({2}), rewards);
  EXPECT_TRUE(next_mcts_state->IsTerminal());  // < action 3 should lead to a
                                               // collision with other agent
  rewards += next_mcts_state->GetTerminalReward();
  EXPECT_NEAR(rewards[0](0), -1000, 0.00001);

  // Checking goal reached: Do multiple steps and expect that goal is reached
  next_mcts_state = mcts_state.Execute(JointAction({0}), rewards);
  for (int i = 0; i < 10000; ++i) {
    if (next_mcts_state->IsTerminal()) {
      break;
    }
    next_mcts_state = next_mcts_state->Execute(JointAction({0}), rewards);
  }
  EXPECT_TRUE(next_mcts_state->IsTerminal());
  bark::world::EvaluatorPtr evaluator_goal_reached(new EvaluatorGoalReached());
  auto terminal_world = next_mcts_state->GetObservedWorld();
  auto results = evaluator_goal_reached->Evaluate(*terminal_world);
  EXPECT_TRUE(boost::get<bool>(results));
}

template <typename T>
class SingleAgentSuite : public ::testing::Test {
 protected:
  SingleAgentSuite()
      : params(new SetterParams()),
        polygon(Pose(0, 0, 0),
                std::vector<Point2d>{Point2d(-2.5, -2.5), Point2d(-2.5, 2.5),
                                     Point2d(2.5, 2.5), Point2d(2.5, -2.5),
                                     Point2d(-2.5, -2.5)}) {
    // Testing single agent behavior
    params->SetBool("BehaviorMvmcts::MultiAgent", false);
    params->SetInt("BehaviorMvmcts::MaxNumIterations", 1000);
    params->SetReal("BehaviorMvmcts::StateParameters::GoalReward", 100);
    BehaviorModelPtr ego_prediction_model(
        new BehaviorMPContinuousActions(params));
    Input u1(2);
    u1 << 0, 0;
    Input u2(2);
    u2 << 5, 0;
    Input u3(2);
    u3 << 0, -1;
    Input u4(2);
    u4 << 0, 1;
    Input u5(2);
    u5 << -3, 0;
    std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
        ->AddMotionPrimitive(u1);
    std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
        ->AddMotionPrimitive(u2);
    std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
        ->AddMotionPrimitive(u3);
    std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
        ->AddMotionPrimitive(u4);
    std::dynamic_pointer_cast<BehaviorMPContinuousActions>(ego_prediction_model)
        ->AddMotionPrimitive(u5);
    BehaviorModelPtr others_prediction_model(
        new BehaviorConstantAcceleration(params));
    prediction_settings =
        PredictionSettings(ego_prediction_model, others_prediction_model);
    behavior = BehaviorModelPtr(
        new T(params, prediction_settings, make_labels(), make_rules(), {}));
  }
  ParamsPtr params;
  Polygon polygon;
  PredictionSettings prediction_settings;
  BehaviorModelPtr behavior;
};

using testing::Types;

typedef Types<BehaviorMvmctsUct, BehaviorMvmctsGreedy> MyTypes;
TYPED_TEST_SUITE(SingleAgentSuite, MyTypes);

TYPED_TEST(SingleAgentSuite, no_agent_in_front_accelerate) {
  // Test if planner accelerates if there is no agent in front

  float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference = 0.0,
        prediction_time_span = 0.2f;
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(
      this->polygon.Translate(Point2d(10, -1.75))));
  // < move the goal polygon into the driving corridor in front of the ego
  // vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto observed_world = make_test_observed_world(
      0, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  Trajectory trajectory =
      this->behavior->Plan(prediction_time_span, observed_world);
  // According to the default motion primitives the best action should be to
  // accelerate without steering (5,0) being action 1 (starting at 0)
  EXPECT_EQ(boost::get<DiscreteAction>(this->behavior->GetLastAction()), 1);
}

TYPED_TEST(SingleAgentSuite, agent_in_front_must_brake) {
  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0,
        prediction_time_span = 0.2f;
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(
      this->polygon.Translate(Point2d(10, -1.75))));
  // < move the goal polygon into the driving corridor in front of the ego
  // vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto observed_world = make_test_observed_world(
      1, rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  Trajectory trajectory =
      this->behavior->Plan(prediction_time_span, observed_world);
  // According to the default motion primitives the best action should be to
  //  brake to avoid crashing the other agent
  EXPECT_EQ(boost::get<DiscreteAction>(this->behavior->GetLastAction()), 4);
}

TYPED_TEST(SingleAgentSuite, agent_in_front_reach_goal) {
  // Test if the planner reaches the goal at some point when agent is slower
  // and in front

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0,
        prediction_time_span = 0.2f;

  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(
      this->polygon.Translate(Point2d(10, -1.75))));
  // < move the goal polygon into the driving corridor in front of the ego
  //  vehicle
  auto goal_definition_ptr =
      std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto world = make_test_world(1, rel_distance, ego_velocity,
                               velocity_difference, goal_definition_ptr);

  world->GetAgents().begin()->second->SetBehaviorModel(this->behavior);

  auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(
      world->GetAgents().begin()->second->GetAgentId());

  bool goal_reached = false;
  for (int i = 0; i < 100; ++i) {
    world->Step(prediction_time_span);
    bool collision_ego =
        boost::get<bool>(evaluator_collision_ego.Evaluate(*world));
    EXPECT_FALSE(collision_ego);
    if (world->GetAgents().begin()->second->AtGoal()) {
      goal_reached = true;
      break;
    }
  }
  EXPECT_TRUE(goal_reached);
}

TYPED_TEST(SingleAgentSuite, change_lane) {
  // Test if the planner reaches the goal at some point when agent is slower
  //  and in front

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference = 2.0,
        prediction_time_span = 0.2f;

  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(
      this->polygon.Translate(Point2d(10, -2))));
  // < move the goal polygon into the driving corridor to the side of the ego
  //  vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionStateLimits>(
      *goal_polygon, std::make_pair<float, float>(-0.2f, 0.2f));

  auto world = make_test_world(0, rel_distance, ego_velocity,
                               velocity_difference, goal_definition_ptr);

  world->GetAgents().begin()->second->SetBehaviorModel(this->behavior);

  auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(
      world->GetAgents().begin()->second->GetAgentId());

  bool goal_reached = false;
  for (int i = 0; i < 20; ++i) {
    world->Step(prediction_time_span);
    LOG(INFO)
        << "State: "
        << world->GetAgents().begin()->second->GetCurrentState().transpose();
    bool collision_ego =
        boost::get<bool>(evaluator_collision_ego.Evaluate(*world));
    EXPECT_FALSE(collision_ego);
    if (world->GetAgents().begin()->second->AtGoal()) {
      goal_reached = true;
      break;
    }
  }
  EXPECT_TRUE(goal_reached);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  //  FLAGS_v = 1;
  return RUN_ALL_TESTS();
}
