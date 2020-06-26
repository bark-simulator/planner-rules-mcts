// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef SRC_MVMCTS_STATE_MULTI_AGENT_HPP_
#define SRC_MVMCTS_STATE_MULTI_AGENT_HPP_

#include <iostream>

#include "ltl/rule_state.h"
#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "mcts/state.h"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/observed_world.hpp"
#include "mvmcts_state_parameters.hpp"

namespace bark {

namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world

namespace models {
namespace behavior {

using namespace mcts;
using ltl::RuleState;
using bark::world::AgentPtr;
using bark::world::objects::AgentId;
using mcts::Reward;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using dynamic::State;
using bark::world::evaluation::BaseLabelFunction;

typedef std::vector<std::shared_ptr<BaseLabelFunction>> LabelEvaluators;
typedef std::unordered_map<AgentId, std::vector<RuleState>>
    MultiAgentRuleState;

class MvmctsState : public mcts::StateInterface<MvmctsState> {
 public:
  MvmctsState(const bark::world::ObservedWorldPtr &observed_world,
                        const MultiAgentRuleState &multi_agent_rule_state,
                        const MvmctsStateParameters *params,
                        const std::vector<AgentIdx> &agent_idx,
                        unsigned int horizon, const LabelEvaluators* label_evaluators);

  std::shared_ptr<MvmctsState> Clone() const;

  std::shared_ptr<MvmctsState> Execute(
      const mcts::JointAction &joint_action,
      std::vector<mcts::Reward> &rewards) const;

  mcts::ActionIdx GetNumActions(mcts::AgentIdx agent_idx) const;

  bool IsTerminal() const;

  const std::vector<mcts::AgentIdx> GetAgentIdx() const;

  std::string PrintState() const;

  std::vector<Reward> GetTerminalReward() const;

  JointReward EvaluateRules();

  const MultiAgentRuleState &GetMultiAgentRuleState() const;

  const ObservedWorldPtr &GetObservedWorld() const;

 private:
  Reward EvaluateRules(const AgentPtr &agent);

  Reward PotentialReward(AgentId agent_id, const State &new_state,
                         const State &current_state) const;
  Reward GetActionCost(const std::shared_ptr<const world::Agent> &agent) const;
  inline float Potential(AgentId agent_id, const State& state) const;
  bool CheckTerminal() const;

  MultiAgentRuleState multi_agent_rule_state_;
  const std::vector<AgentIdx> agent_idx_;
  const MvmctsStateParameters *state_params_;
  const unsigned int horizon_;
  const ObservedWorldPtr observed_world_;
  bool is_terminal_state_;
  const LabelEvaluators* label_evaluators_;
};
}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // SRC_MVMCTS_STATE_MULTI_AGENT_HPP_
