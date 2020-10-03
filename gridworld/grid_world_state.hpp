// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch, Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_GRID_WORLD_STATE_HPP_
#define GRIDWORLD_GRID_WORLD_STATE_HPP_

#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gridworld/common.hpp"
#include "gridworld/grid_world_state_parameter.h"
#include "gridworld/label_evaluator/evaluator_label_base.h"
#include "ltl/rule_monitor.h"

using namespace mvmcts;
using namespace ltl;

typedef std::vector<std::multimap<Rule, RuleState>> RuleStateMap;

class GridWorldState : public mvmcts::StateInterface<GridWorldState> {
 public:
  typedef Actions ActionType;

  GridWorldState(
      RuleStateMap rule_state_map,
      std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
      const GridWorldStateParameter &parameters);

  GridWorldState(
      std::vector<AgentState> agent_states, const bool terminal,
      RuleStateMap rule_state_map,
      std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
      GridWorldStateParameter parameters, int depth,
      std::vector<bool> terminal_agents);
  ~GridWorldState() override = default;

  std::shared_ptr<GridWorldState> Clone() const;

  std::shared_ptr<GridWorldState> Execute(const JointAction &joint_action,
                                          std::vector<Reward> &rewards) const;

  std::vector<Reward> GetTerminalReward() const;

  template <typename ActionType = Actions>
  ActionType GetLastAction(const AgentIdx &agent_idx) const;

  ActionIdx GetNumActions(AgentIdx agent_idx) const;

  bool IsTerminal() const;

  std::vector<AgentIdx> GetAgentIdx() const;

  bool EgoGoalReached() const;

  int GetEgoPos() const;

  const std::vector<AgentState> &GetAgentStates() const;

  const RuleStateMap &GetRuleStateMap() const;

  void ResetDepth();

  std::string PrintState() const;

  const GridWorldStateParameter &GetParameters() const;

  EvaluationMap GetAgentLabels(AgentIdx agent_idx) const;

 private:
  std::vector<AgentState> Step(const JointAction &joint_action) const;
  Reward GetActionCost(ActionIdx action, AgentIdx agent_idx) const;
  Reward GetShapingReward(const AgentState &agent_state) const;
  void SetCollisionPositions(std::vector<AgentState> *agent_states) const;

  std::vector<AgentState> agent_states_;
  bool terminal_;
  RuleStateMap rule_state_map_;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator_;
  int depth_;
  GridWorldStateParameter parameters_;
  std::vector<bool> terminal_agents_;
};

#endif  // GRIDWORLD_GRID_WORLD_STATE_HPP_
