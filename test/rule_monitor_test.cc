// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "ltl/rule_monitor.h"
#include "ltl/label.h"
#include "ltl/rule_state.h"

using ltl::RuleMonitor;
using ltl::EvaluationMap;
using ltl::RuleState;
using ltl::Label;

using RuleMonitorSPtr = RuleMonitor::RuleMonitorSPtr;

TEST(rule_monitor_test, transit_test) {
  RuleMonitorSPtr eval = RuleMonitor::make_rule("F (G a)", -1.0f, 0);
  RuleState state = eval->make_rule_state()[0];
  EvaluationMap map;
  map[Label("a")] = false;
  ASSERT_EQ(0.0f, state.get_automaton()->evaluate(map, state));
  ASSERT_EQ(0.0f, state.get_automaton()->evaluate(map, state));
  map[Label("a")] = true;
  ASSERT_EQ(0.0f, state.get_automaton()->evaluate(map, state));
  ASSERT_EQ(0.0f, state.get_automaton()->evaluate(map, state));
  map[Label("a")] = false;
  ASSERT_EQ(0.0f, state.get_automaton()->evaluate(map, state));
  ASSERT_EQ(-1.0f, state.get_automaton()->get_final_reward(state));
}