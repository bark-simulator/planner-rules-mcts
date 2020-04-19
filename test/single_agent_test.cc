// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "mcts/mcts.h"
#include "modules/commons/params/default_params.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/tests/make_test_world.hpp"
#include "modules/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "modules/world/evaluation/evaluator_goal_reached.hpp"
#include "modules/world/evaluation/labels/generic_ego_label_evaluator.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits.hpp"

using namespace modules::models::behavior;
using namespace mcts;
using modules::models::tests::make_test_observed_world;
using modules::models::tests::make_test_world;
using modules::world::prediction::PredictionSettings;
using modules::models::dynamic::SingleTrackModel;
using modules::models::dynamic::Input;
using modules::world::ObservedWorldPtr;
using modules::commons::DefaultParams;
using modules::commons::SetterParams;
using modules::geometry::Polygon;
using modules::geometry::Point2d;
using modules::geometry::Pose;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::models::dynamic::Trajectory;
using modules::world::evaluation::EvaluatorGoalReached;
using modules::world::evaluation::EvaluatorCollisionEgoAgent;
using modules::world::evaluation::GenericLabelEvaluator;
using modules::world::evaluation::GenericEgoLabelEvaluator;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::world::prediction::PredictionSettings;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::world::prediction::PredictionSettings;

LabelEvaluators make_labels() {
  LabelEvaluators labels;
  labels.emplace_back(new GenericEgoLabelEvaluator<EvaluatorCollisionEgoAgent>(
      "collision_ego"));
  labels.emplace_back(
      new GenericEgoLabelEvaluator<EvaluatorGoalReached>("goal_reached"));
  return labels;
}

std::vector<std::shared_ptr<RuleMonitor>> make_rules() {
  std::vector<std::shared_ptr<RuleMonitor>> rules;
  rules.emplace_back(RuleMonitor::make_rule("G !collision_ego", -1000,
                                                 0));
  rules.emplace_back(RuleMonitor::make_rule("G !collision_corridor", -1000,
                                                 0));
  rules.emplace_back(RuleMonitor::make_rule(
      "F goal_reached", 0, 0, 1.0, 1.0));
  return rules;
}

std::vector<RuleState> make_rule_states(
    const std::vector<std::shared_ptr<RuleMonitor>> &rules) {
  std::vector<RuleState> rule_states;
  for (const auto &rule : rules) {
    auto rs = rule->make_rule_state();
    rule_states.insert(rule_states.end(), rs.begin(), rs.end());
  }
  return rule_states;
}

TEST(single_agent_mcts_state, execute) {
  // Setup prediction models for ego agent and other agents   
  DefaultParams params;
  DynamicModelPtr dyn_model(new SingleTrackModel(&params));
  BehaviorModelPtr ego_prediction_model(new BehaviorMotionPrimitives(dyn_model, &params));
  float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.2f;
  Input u1(2);  u1 << 0, 0;
  Input u2(2);  u2 << 50, 1; //  < crazy action to drive out of the corridors
  Input u3(2);  u3 << (rel_distance+4)*2/(prediction_time_span*prediction_time_span), 0; //  < action to drive into other agent with a single step
                                                                                        //   (4m vehicle length assumed)
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u1);
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u2);
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u3);
  BehaviorModelPtr others_prediction_model(new BehaviorConstantVelocity(&params));
  PredictionSettings prediction_settings(ego_prediction_model, others_prediction_model);

  // Create an observed world with specific goal definition and the corresponding mcts state
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(200,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);

  auto observed_world = std::dynamic_pointer_cast<ObservedWorld>(make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr).Clone());
  observed_world->SetupPrediction(prediction_settings);

  observed_world->AddLabels(make_labels());
  MvmctsStateParameters state_params(&params);
  MvmctsStateSingleAgent mcts_state(observed_world, make_rule_states(make_rules()), &state_params);

  std::vector<mcts::Reward> rewards;
  auto next_mcts_state = mcts_state.execute(JointAction({0}), rewards);
  rewards += next_mcts_state->get_final_reward();
  // Checking reward of zero if we are neither colliding or at goal
  EXPECT_FALSE(next_mcts_state->is_terminal()); // < make test world is defined in such a way that
                                                // a long driving corridor along x exists
                                                // no collision should occur after one action
  EXPECT_NEAR(rewards[0](0), 0 , 0.00001);

  // Checking collision with corridor
  next_mcts_state = next_mcts_state->execute(JointAction({1}), rewards);
  next_mcts_state = next_mcts_state->execute(JointAction({1}), rewards);
  next_mcts_state = next_mcts_state->execute(JointAction({1}), rewards);
  next_mcts_state = next_mcts_state->execute(JointAction({1}), rewards);
  next_mcts_state = next_mcts_state->execute(JointAction({1}), rewards);
  next_mcts_state = next_mcts_state->execute(JointAction({1}), rewards);
  next_mcts_state = next_mcts_state->execute(JointAction({1}), rewards);

  EXPECT_TRUE(next_mcts_state->is_terminal()); // < crazy action should lead to a collision with driving corridor
  rewards += next_mcts_state->get_final_reward();
  EXPECT_NEAR(rewards[0](0), -1000 , 0.00001);

  // Checking collision with other agent ( use initial state again)
  next_mcts_state = mcts_state.execute(JointAction({2}), rewards);
  EXPECT_TRUE(next_mcts_state->is_terminal()); // < action 3 should lead to a collision with other agent
  rewards += next_mcts_state->get_final_reward();
  EXPECT_NEAR(rewards[0](0), -1000 , 0.00001);


  // Checking goal reached: Do multiple steps and expect that goal is reached
  bool reached = false;
  next_mcts_state = mcts_state.execute(JointAction({0}), rewards);
  reached = next_mcts_state->is_terminal();
  for (int i = 0; i < 10000; ++i) {
    if(next_mcts_state->is_terminal()) {
      reached = true;
      break;
    }
    next_mcts_state = next_mcts_state->execute(JointAction({0}), rewards);
  }
  EXPECT_TRUE(reached);
  rewards += next_mcts_state->get_final_reward();
  EXPECT_NEAR(rewards[0](0), 1 , 0.00001); // < reward should be one when reaching the goal
}


TEST(single_agent_mcts_state, execute_goal_reached_state_limits) {
  // Setup prediction models for ego agent and other agents   
  DefaultParams params;
  DynamicModelPtr dyn_model(new SingleTrackModel(&params));
  BehaviorModelPtr ego_prediction_model(new BehaviorMotionPrimitives(dyn_model, &params));
  float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.2f;
  Input u1(2);  u1 << 0, 0;
  Input u2(2);  u2 << 50, 3; //  < crazy action to drive out of the corridors
  Input u3(2);  u3 << (rel_distance+4)*2/(prediction_time_span*prediction_time_span), 0; //  < action to drive into other agent with a single step
                                                                                        //   (4m vehicle length assumed)
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u1);
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u2);
  std::dynamic_pointer_cast<BehaviorMotionPrimitives>(ego_prediction_model)->AddMotionPrimitive(u3);
  BehaviorModelPtr others_prediction_model(new BehaviorConstantVelocity(&params));
  PredictionSettings prediction_settings(ego_prediction_model, others_prediction_model);

  // Create an observed world with specific goal definition and the corresponding mcts state
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(5, 2), Point2d(5, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10,-1)))); // < move the state limit region to the front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionStateLimits>(*goal_polygon, std::make_pair<float, float>(-0.2f,0.2f));

  auto observed_world = std::dynamic_pointer_cast<ObservedWorld>(make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr).Clone());
  observed_world->SetupPrediction(prediction_settings);

  observed_world->AddLabels(make_labels());
  MvmctsStateParameters state_params(&params);
  MvmctsStateSingleAgent mcts_state(observed_world, make_rule_states(make_rules()), &state_params);

  // Checking goal reached: Do multiple steps and expect that goal is reached
  std::vector<mcts::Reward> rewards;
  bool reached = false;
  auto next_mcts_state = mcts_state.execute(JointAction({0}), rewards);
  reached = next_mcts_state->is_terminal();
  for (int i = 0; i < 10000; ++i) {
    if(next_mcts_state->is_terminal()) {
      reached = true;
      break;
    }
    next_mcts_state = next_mcts_state->execute(JointAction({0}), rewards);
  }
  rewards += next_mcts_state->get_final_reward();
  EXPECT_TRUE(reached);
  EXPECT_NEAR(rewards[0](0), 1 , 0.00001); // < reward should be one when reaching the goal
}

TEST(behavior_uct_single_agent, no_agent_in_front_accelerate) {
  // Test if uct planner accelerates if there is no agent in front
  SetterParams params;
  params.SetInt("BehaviorMCTSSingleAgent::MaxNumIterations", 10000);
  params.SetInt("BehaviorMCTSSingleAgent::MaxSearchTime", 20000);
  params.SetInt("BehaviorMCTSSingleAgent::RandomSeed", 1000);
  params.SetBool("BehaviorMCTSSingleAgent::DumpTree", true);
  params.SetListListFloat("BehaviorMCTSSingleAgent::MotionPrimitiveInputs", {{0,0}, {5,0}, {0,-1}, {0, 1}, {-3,0}});
  params.SetReal("BehaviorMCTSSingleAgent::DiscountFactor", 0.9);
  params.SetReal("BehaviorMCTSSingleAgent::UCTExplorationConstant", 0.7);
  params.SetReal("BehaviorMCTSSingleAgent::MaxSearchTimeRandomHeuristic", 100);
  params.SetReal("BehaviorMCTSSingleAgent::MaxNumIterationsRandomHeuristic",
                  1000);
  params.SetReal("BehaviorMCTSSingleAgent::ReturnLowerBound", -1000);
  params.SetReal("BehaviorMCTSSingleAgent::ReturnUpperBound", 100);

  float ego_velocity = 5.0, rel_distance = 7.0, velocity_difference=0.0, prediction_time_span=0.2f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  observed_world.AddLabels(make_labels());
  BehaviorUCTSingleAgent behavior_uct(&params);
  behavior_uct.add_rules(make_rules());

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  // According to the default motion primitives the best action should be to accelerate extremely without steering (20,0) being action 1 (starting at 0)
  EXPECT_EQ(boost::get<DiscreteAction>(behavior_uct.GetLastAction()), 1);

}

TEST(behavior_uct_single_agent, agent_in_front_must_brake) {
  // Test if uct planner brakes when slow agent is directly in front
  SetterParams params;
  params.SetInt("BehaviorMCTSSingleAgent::MaxNumIterations", 10000);
  params.SetInt("BehaviorMCTSSingleAgent::MaxSearchTime", 20000);
  params.SetInt("BehaviorMCTSSingleAgent::RandomSeed", 1000);
  params.SetBool("BehaviorMCTSSingleAgent::DumpTree", true);
  params.SetListListFloat("BehaviorMCTSSingleAgent::MotionPrimitiveInputs", {{0,0}, {5,0}, {0,-1}, {0, 1}, {-3,0}});
  params.SetReal("BehaviorMCTSSingleAgent::DiscountFactor", 0.9);
  params.SetReal("BehaviorMCTSSingleAgent::UCTExplorationConstant", 0.7);
  params.SetReal("BehaviorMCTSSingleAgent::MaxSearchTimeRandomHeuristic",1);
  params.SetReal("BehaviorMCTSSingleAgent::MaxNumIterationsRandomHeuristic", 100);
  params.SetReal("BehaviorMCTSSingleAgent::ReturnLowerBound", -1000);
  params.SetReal("BehaviorMCTSSingleAgent::ReturnUpperBound", 100);

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference=2.0, prediction_time_span=0.2f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto observed_world = make_test_observed_world(1,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);
  observed_world.AddLabels(make_labels());

  BehaviorUCTSingleAgent behavior_uct(&params);
  behavior_uct.add_rules(make_rules());

  Trajectory trajectory = behavior_uct.Plan(prediction_time_span, observed_world);
  // According to the default motion primitives the best action should be to brake to avoid crahsing the other agent
  EXPECT_EQ(boost::get<DiscreteAction>(behavior_uct.GetLastAction()), 4);

}

TEST(behavior_uct_single_agent, agent_in_front_reach_goal) {
  // Test if the planner reaches the goal at some point when agent is slower and in front
  SetterParams params;
  params.SetInt("BehaviorMCTSSingleAgent::MaxNumIterations", 10000);
  params.SetInt("BehaviorMCTSSingleAgent::MaxSearchTime", 2000);
  params.SetInt("BehaviorMCTSSingleAgent::RandomSeed", 1000);
  params.SetBool("BehaviorMCTSSingleAgent::DumpTree", true);
  params.SetListListFloat("BehaviorMCTSSingleAgent::MotionPrimitiveInputs", {{0,0}, {5,0}, {0,-1}, {0, 1}, {-3,0}});
  params.SetReal("BehaviorMCTSSingleAgent::DiscountFactor", 0.9);
  params.SetReal("BehaviorMCTSSingleAgent::UCTExplorationConstant", 0.7);
  params.SetReal("BehaviorMCTSSingleAgent::MaxSearchTimeRandomHeuristic",1);
  params.SetReal("BehaviorMCTSSingleAgent::MaxNumIterationsRandomHeuristic", 100);
  params.SetReal("BehaviorMCTSSingleAgent::ReturnLowerBound", -1000);
  params.SetReal("BehaviorMCTSSingleAgent::ReturnUpperBound", 100);

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference=2.0, prediction_time_span=0.2f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(2, 2), Point2d(2, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10,0)))); // < move the goal polygon into the driving corridor in front of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionPolygon>(*goal_polygon);
  
  auto world = make_test_world(1,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  BehaviorModelPtr behavior_uct(new BehaviorUCTSingleAgent(&params));
  std::dynamic_pointer_cast<BehaviorUCTSingleAgent>(behavior_uct)
      ->add_labels(make_labels());
  std::dynamic_pointer_cast<BehaviorUCTSingleAgent>(behavior_uct)
      ->add_rules(make_rules());

  world->GetAgents().begin()->second->SetBehaviorModel(behavior_uct);

  auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(world->GetAgents().begin()->second->GetAgentId());
        

  bool goal_reached = false;
  for(int i =0; i<100; ++i) {
    world->Step(prediction_time_span);
    bool collision_ego = boost::get<bool>(evaluator_collision_ego.Evaluate(*world));
    EXPECT_FALSE(collision_ego);
    if(world->GetAgents().begin()->second->AtGoal()) {
      goal_reached = true;
      break;
    }
  }
  EXPECT_TRUE(goal_reached);

}

TEST(behavior_uct_single_agent, change_lane) {
  // Test if the planner reaches the goal at some point when agent is slower and in front
  SetterParams params;
  params.SetInt("BehaviorMCTSSingleAgent::MaxNumIterations", 10000);
  params.SetInt("BehaviorMCTSSingleAgent::MaxSearchTime", 20000);
  params.SetInt("BehaviorMCTSSingleAgent::RandomSeed", 1000);
  params.SetBool("BehaviorMCTSSingleAgent::DumpTree", true);
  params.SetListListFloat("BehaviorMCTSSingleAgent::MotionPrimitiveInputs", {{0,0}, {1,0}, {0,-0.27}, {0, 0.27}, {0,-0.17}, {0, 0.17}, {-1,0}});
  params.SetReal("BehaviorMCTSSingleAgent::DiscountFactor", 0.9);
  params.SetReal("BehaviorMCTSSingleAgent::UCTExplorationConstant", 0.7);
  params.SetReal("BehaviorMCTSSingleAgent::MaxSearchTimeRandomHeuristic",
                  1000);
  params.SetReal("BehaviorMCTSSingleAgent::MaxNumIterationsRandomHeuristic",
                  1000);
  params.SetReal("BehaviorMCTSSingleAgent::ReturnLowerBound", -1000);
  params.SetReal("BehaviorMCTSSingleAgent::ReturnUpperBound", 100);

  float ego_velocity = 5.0, rel_distance = 2.0, velocity_difference=2.0, prediction_time_span=0.2f;
  Polygon polygon(Pose(1, 1, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(5, 2), Point2d(5, 0), Point2d(0, 0)});
  std::shared_ptr<Polygon> goal_polygon(std::dynamic_pointer_cast<Polygon>(polygon.Translate(Point2d(10,-2)))); // < move the goal polygon into the driving corridor to the side of the ego vehicle
  auto goal_definition_ptr = std::make_shared<GoalDefinitionStateLimits>(*goal_polygon, std::make_pair<float, float>(-0.2f, 0.2f));
  
  auto world = make_test_world(0,rel_distance, ego_velocity, velocity_difference, goal_definition_ptr);

  BehaviorModelPtr behavior_uct(new BehaviorUCTSingleAgent(&params));

  std::dynamic_pointer_cast<BehaviorUCTSingleAgent>(behavior_uct)
      ->add_labels(make_labels());
  std::dynamic_pointer_cast<BehaviorUCTSingleAgent>(behavior_uct)
      ->add_rules(make_rules());

  world->GetAgents().begin()->second->SetBehaviorModel(behavior_uct);

  auto evaluator_collision_ego = EvaluatorCollisionEgoAgent(world->GetAgents().begin()->second->GetAgentId());
        

  bool goal_reached = false;
  for(int i =0; i<20; ++i) {
    world->Step(prediction_time_span);
    LOG(INFO) << "State: " << world->GetAgents().begin()->second->GetCurrentState().transpose();
    bool collision_ego = boost::get<bool>(evaluator_collision_ego.Evaluate(*world));
    EXPECT_FALSE(collision_ego);
    if(world->GetAgents().begin()->second->AtGoal()) {
      goal_reached = true;
      break;
    }
  }
  EXPECT_TRUE(goal_reached);

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  FLAGS_v = 1;
  return RUN_ALL_TESTS();

}
