// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_PLANNER_RULES_MCTS_RULES_MCTS_STATE_HPP_
#define MODULES_MODELS_BEHAVIOR_PLANNER_RULES_MCTS_RULES_MCTS_STATE_HPP_

#include <iostream>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/observed_world.hpp"
#include "ltl/rule_state.h"
#include "mvmcts/state.h"
#include "rules_mcts_state_parameters.hpp"

namespace bark {

namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world

namespace models {
namespace behavior {

using namespace mvmcts;
using bark::world::AgentPtr;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using bark::world::evaluation::BaseLabelFunction;
using bark::world::objects::AgentId;
using dynamic::State;
using ltl::RuleState;

typedef std::vector<std::shared_ptr<BaseLabelFunction>> LabelEvaluators;
typedef std::unordered_map<AgentId, std::vector<RuleState>> MultiAgentRuleState;

class MvmctsState : public StateInterface<MvmctsState> {
 public:
  MvmctsState(const bark::world::ObservedWorldPtr& observed_world,
              const MultiAgentRuleState& multi_agent_rule_state,
              const RulesMctsStateParameters* params,
              const std::vector<AgentIdx>& agent_idx, unsigned int horizon,
              const LabelEvaluators* label_evaluators);

  std::shared_ptr<MvmctsState> Clone() const;

  std::shared_ptr<MvmctsState> Execute(const JointAction& joint_action,
                                       std::vector<Reward>& rewards) const;

  ActionIdx GetNumActions(AgentIdx agent_idx) const;

  bool IsTerminal() const;

  const std::vector<AgentIdx> GetAgentIdx() const;

  std::string PrintState() const;

  std::vector<Reward> GetTerminalReward() const;

  JointReward EvaluateRules();

  const MultiAgentRuleState& GetMultiAgentRuleState() const;

  const ObservedWorldPtr& GetObservedWorld() const;

 private:
  Reward EvaluateRules(const AgentPtr& agent);

  Reward PotentialReward(AgentId agent_id, const State& new_state,
                         const State& current_state) const;
  Reward GetActionCost(const std::shared_ptr<const world::Agent>& agent) const;
  inline double Potential(AgentId agent_id, const State& state) const;
  bool CheckTerminal() const;

  MultiAgentRuleState multi_agent_rule_state_;
  const std::vector<AgentIdx> agent_idx_;
  const RulesMctsStateParameters* state_params_;
  const unsigned int horizon_;
  const ObservedWorldPtr observed_world_;
  bool is_terminal_state_;
  const LabelEvaluators* label_evaluators_;
};
}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_PLANNER_RULES_MCTS_RULES_MCTS_STATE_HPP_
