// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_MVMCTS_STATE_MULTI_AGENT_HPP_
#define MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_MVMCTS_STATE_MULTI_AGENT_HPP_

#include <iostream>

#include "ltl/rule_state.h"
#include "mcts/state.h"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/observed_world.hpp"
#include "mvmcts_state_parameters.hpp"

namespace modules {

namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world

namespace models {
namespace behavior {

using namespace mcts;
using ltl::RuleState;
using modules::world::AgentPtr;
using modules::world::objects::AgentId;
using mcts::Reward;
using modules::world::ObservedWorld;
using modules::world::ObservedWorldPtr;
using dynamic::State;

typedef std::unordered_map<AgentId, std::vector<RuleState>>
    MultiAgentRuleState;

class MvmctsStateMultiAgent
    : public mcts::StateInterface<MvmctsStateMultiAgent> {
 public:
  MvmctsStateMultiAgent(const modules::world::ObservedWorldPtr &observed_world,
                        const MultiAgentRuleState &multi_agent_rule_state,
                        const MvmctsStateParameters *params,
                        const std::vector<AgentIdx> &agent_idx,
                        unsigned int horizon);

  std::shared_ptr<MvmctsStateMultiAgent> clone() const;

  std::shared_ptr<MvmctsStateMultiAgent> execute(
      const mcts::JointAction &joint_action,
      std::vector<mcts::Reward> &rewards) const;

  mcts::ActionIdx get_num_actions(mcts::AgentIdx agent_idx) const;

  bool is_terminal() const;

  const std::vector<mcts::AgentIdx> get_agent_idx() const;

  std::string sprintf() const;

  std::vector<Reward> get_final_reward() const;

  JointReward evaluate_rules();

  const MultiAgentRuleState &get_multi_agent_rule_state() const;

  const ObservedWorldPtr &GetObservedWorld() const;

 private:
  Reward evaluate_rules(const AgentPtr &agent);

  Reward PotentialReward(AgentId agent_id, const State &new_state,
                         const State &current_state) const;
  Reward get_action_cost(const std::shared_ptr<const world::Agent> &agent) const;
  inline float Potential(AgentId agent_id, const State& state) const;
  bool check_terminal() const;

  MultiAgentRuleState multi_agent_rule_state_;
  const std::vector<AgentIdx> agent_idx_;
  const MvmctsStateParameters *state_params_;
  const unsigned int horizon_;
  const ObservedWorldPtr observed_world_;
  bool is_terminal_state_;
};
}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_PLANNER_MCTS_MVMCTS_STATE_MULTI_AGENT_HPP_
