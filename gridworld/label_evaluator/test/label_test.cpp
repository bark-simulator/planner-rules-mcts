// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gridworld/label_evaluator/evaluator_label_at_position.h"
#include "gridworld/label_evaluator/evaluator_label_collision.h"
#include "gtest/gtest.h"

TEST(label_test, at_position_test) {
  EvaluatorLabelAtPosition evaluator("position", 5);
  AgentState agent;

  // On position
  agent.x_pos = 5;
  agent.last_action = 1;
  agent.lane = 1;
  World w(agent, {});
  EXPECT_TRUE(evaluator.evaluate(w)[0].second);

  // Step over left -> right
  agent.x_pos = 6;
  agent.last_action = 2;
  agent.lane = 1;
  w = World(agent, {});
  EXPECT_TRUE(evaluator.evaluate(w)[0].second);

  // Step over right -> left
  agent.x_pos = 4;
  agent.last_action = -2;
  agent.lane = 1;
  w = World(agent, {});
  EXPECT_TRUE(evaluator.evaluate(w)[0].second);

  // On position in last time step, right -> left
  agent.x_pos = 4;
  agent.last_action = -1;
  agent.lane = 1;
  w = World(agent, {});
  EXPECT_FALSE(evaluator.evaluate(w)[0].second);

  // On position in last time step, left -> right
  agent.x_pos = 6;
  agent.last_action = 1;
  agent.lane = 1;
  w = World(agent, {});
  EXPECT_FALSE(evaluator.evaluate(w)[0].second);

  // Not on position
  agent.x_pos = 7;
  agent.last_action = 1;
  agent.lane = 1;
  w = World(agent, {});
  EXPECT_FALSE(evaluator.evaluate(w)[0].second);

  // Not on position
  agent.x_pos = 3;
  agent.last_action = -1;
  agent.lane = 1;
  w = World(agent, {});
  EXPECT_FALSE(evaluator.evaluate(w)[0].second);
}

TEST(label_test, collision_test) {
  EvaluatorLabelCollision evaluator("collision", 0);
  AgentState ego;
  AgentState other;
  World w(ego, {other});
  World wo(other, {ego});
  EXPECT_TRUE(evaluator.evaluate(w)[0].second);
  EXPECT_TRUE(evaluator.evaluate(wo)[0].second);

  ego.x_pos = 5;
  ego.last_action = 2;
  ego.lane = 1;
  other.x_pos = 4;
  other.last_action = -1;
  other.lane = 1;
  World w1(ego, {other});
  World w2(other, {ego});
  EXPECT_TRUE(evaluator.evaluate(w1)[0].second);
  EXPECT_TRUE(evaluator.evaluate(w2)[0].second);

  ego.x_pos = 5;
  ego.last_action = 2;
  ego.lane = 1;
  other.x_pos = 4;
  other.last_action = 1;
  other.lane = 1;
  w1.first = ego;
  w1.second = {other};
  w2.first = other;
  w2.second = {ego};
  EXPECT_TRUE(evaluator.evaluate(w1)[0].second);
  EXPECT_TRUE(evaluator.evaluate(w2)[0].second);

  ego.x_pos = 5;
  ego.last_action = 2;
  ego.lane = 1;
  other.x_pos = 4;
  other.last_action = 2;
  other.lane = 1;
  w1.first = ego;
  w1.second = {other};
  w2.first = other;
  w2.second = {ego};
  EXPECT_TRUE(evaluator.evaluate(w1)[0].second);
  EXPECT_TRUE(evaluator.evaluate(w2)[0].second);

  ego.x_pos = 5;
  ego.last_action = 2;
  ego.lane = 1;
  other.x_pos = 3;
  other.last_action = 1;
  other.lane = 1;
  w1.first = ego;
  w1.second = {other};
  w2.first = other;
  w2.second = {ego};
  EXPECT_FALSE(evaluator.evaluate(w1)[0].second);
  EXPECT_FALSE(evaluator.evaluate(w2)[0].second);

  ego.x_pos = 5;
  ego.last_action = 2;
  ego.lane = 1;
  other.x_pos = 5;
  other.last_action = 1;
  other.lane = 0;
  w1.first = ego;
  w1.second = {other};
  w2.first = other;
  w2.second = {ego};
  EXPECT_FALSE(evaluator.evaluate(w1)[0].second);
  EXPECT_FALSE(evaluator.evaluate(w2)[0].second);

  ego.x_pos = 1;
  ego.last_action = 0;
  ego.lane = 1;
  other.x_pos = 1;
  other.last_action = 0;
  other.lane = 1;
  w1.first = ego;
  w1.second = {other};
  w2.first = other;
  w2.second = {ego};
  EXPECT_TRUE(evaluator.evaluate(w1)[0].second);
  EXPECT_TRUE(evaluator.evaluate(w2)[0].second);

  ego.x_pos = 14;
  ego.last_action = 2;
  ego.lane = 0;
  other.x_pos = 13;
  other.last_action = 0;
  other.lane = 0;
  w1.first = ego;
  w1.second = {other};
  w2.first = other;
  w2.second = {ego};
  EXPECT_TRUE(evaluator.evaluate(w1)[0].second);
  EXPECT_TRUE(evaluator.evaluate(w2)[0].second);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
