// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch, Julian Bernhard
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <algorithm>
#include <tuple>
#include <utility>

#include "gridworld/grid_world_state.hpp"
#include "gridworld/label_evaluator/evaluator_label_collision.h"

GridWorldState::GridWorldState(
    RuleStateMap rule_state_map,
    std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
    const GridWorldStateParameter &parameters)
    : agent_states_(parameters.num_other_agents + 1),
      terminal_(false),
      rule_state_map_(std::move(rule_state_map)),
      label_evaluator_(std::move(label_evaluator)),
      depth_(0),
      parameters_(parameters),
      terminal_agents_(parameters.num_other_agents + 1, false) {
  for (auto &state : agent_states_) {
    state = AgentState();
  }
}

GridWorldState::GridWorldState(
    std::vector<AgentState> agent_states, const bool terminal,
    RuleStateMap rule_state_map,
    std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluator,
    GridWorldStateParameter parameters, int depth,
    std::vector<bool> terminal_agents)
    : agent_states_(std::move(agent_states)),
      terminal_(terminal),
      rule_state_map_(std::move(rule_state_map)),
      label_evaluator_(std::move(label_evaluator)),
      depth_(depth),
      parameters_(std::move(parameters)),
      terminal_agents_(std::move(terminal_agents)) {}

std::shared_ptr<GridWorldState> GridWorldState::Execute(
    const JointAction &joint_action, std::vector<Reward> &rewards) const {
  EvaluationMap labels;
  RuleStateMap next_automata(rule_state_map_);
  World next_world;
  rewards.resize(parameters_.num_other_agents + 1);

  // CALCULATE SUCCESSOR AGENT STATES
  std::vector<AgentState> next_agent_states = Step(joint_action);

  // REWARD GENERATION
  // For each agent
  std::vector<bool> agent_terminal(agent_states_.size(), false);
  for (size_t agent_idx = 0; agent_idx < rewards.size(); ++agent_idx) {
    if (terminal_agents_[agent_idx]) {
      agent_terminal[agent_idx] = true;
      continue;
    }
    // Labeling
    std::vector<AgentState> next_other_agents(next_agent_states);
    next_other_agents.erase(next_other_agents.begin() + agent_idx);
    // Create perspective from current agent
    next_world = World(next_agent_states[agent_idx], next_other_agents);

    for (const auto &le : label_evaluator_) {
      auto new_labels = le->evaluate(next_world);
      labels.insert(new_labels.begin(), new_labels.end());
    }
    rewards[agent_idx] = Reward::Zero(parameters_.reward_vec_size);

    // Automata transit
    for (auto &aut : (next_automata[agent_idx])) {
      rewards[agent_idx](aut.second.GetPriority()) +=
          aut.second.GetAutomaton()->Evaluate(labels, aut.second);
    }

    rewards[agent_idx] += GetActionCost(joint_action[agent_idx], agent_idx);
    agent_terminal[agent_idx] = agent_terminal[agent_idx] ||
                                labels[Label("collision")] ||
                                (depth_ + 1 >= parameters_.terminal_depth_);
    labels.clear();
  }  // End for each agent
  return std::make_shared<GridWorldState>(
      next_agent_states, agent_terminal[0], next_automata, label_evaluator_,
      parameters_, depth_ + 1, agent_terminal);
}
std::vector<AgentState> GridWorldState::Step(
    const JointAction &joint_action) const {
  std::vector<AgentState> next_agent_states(agent_states_.size());
  for (size_t i = 0; i < this->agent_states_.size(); ++i) {
    if (this->terminal_agents_[i]) {
      next_agent_states[i] = this->agent_states_[i];
    } else {
      const auto &old_state = this->agent_states_[i];
      int new_x =
          old_state.x_pos + this->parameters_.action_map[joint_action[i]];
      int new_lane = old_state.lane;
      if (new_x >= parameters_.merging_point) {
        // merge all agents to the same lane in and
        // after merging point.
        new_lane = 0;
      } else if (new_x < parameters_.merging_point) {
        // When driving backwards, restore the initial lane
        new_lane = old_state.init_lane;
      }
      next_agent_states[i] = old_state;
      next_agent_states[i].x_pos = new_x;
      next_agent_states[i].last_action =
          this->parameters_.action_map[joint_action[i]];
      next_agent_states[i].lane = new_lane;
    }
  }
  SetCollisionPositions(&next_agent_states);
  return next_agent_states;
}
Reward GridWorldState::GetActionCost(ActionIdx action,
                                     AgentIdx agent_idx) const {
  Reward reward = Reward::Zero(parameters_.reward_vec_size);
  reward(reward.rows() - 1) += -std::abs(parameters_.action_map[action] -
                                         static_cast<int>(Actions::FORWARD)) *
                               parameters_.speed_deviation_weight;
  reward(reward.rows() - 1) +=
      -std::pow(
          parameters_.action_map[action] - agent_states_[agent_idx].last_action,
          2) *
      parameters_.acceleration_weight;
  return reward;
}
Reward GridWorldState::GetShapingReward(const AgentState &agent_state) const {
  Reward reward = Reward::Zero(parameters_.reward_vec_size);
  // Potential for goal distance
  reward(reward.rows() - 1) +=
      -parameters_.potential_weight *
      std::abs(parameters_.ego_goal_reached_position - agent_state.x_pos);
  return reward;
}
std::vector<Reward> GridWorldState::GetTerminalReward() const {
  std::vector<Reward> rewards(agent_states_.size(),
                              Reward::Zero(parameters_.reward_vec_size));
  for (size_t agent_idx = 0; agent_idx < rewards.size(); ++agent_idx) {
    // Automata transit
    for (const auto &aut : (rule_state_map_[agent_idx])) {
      rewards[agent_idx](aut.second.GetPriority()) +=
          aut.second.GetAutomaton()->FinalTransit(aut.second);
    }
  }
  return rewards;
}
bool GridWorldState::IsTerminal() const { return terminal_; }
ActionIdx GridWorldState::GetNumActions(AgentIdx agent_idx) const {
  if (terminal_agents_[agent_idx]) {
    return static_cast<ActionIdx>(1);
  }
  return static_cast<ActionIdx>(parameters_.action_map.size());
}
std::vector<AgentIdx> GridWorldState::GetAgentIdx() const {
  std::vector<AgentIdx> agent_idx(parameters_.num_other_agents + 1);
  std::iota(agent_idx.begin(), agent_idx.end(), 0);
  return agent_idx;  // adapt to number of agents
}
std::string GridWorldState::PrintState() const {
  std::stringstream ss;
  ss << "Ego: x=" << agent_states_[ego_agent_idx].x_pos;
  for (size_t i = 1; i < agent_states_.size(); ++i) {
    ss << ", Ag" << i << ": x=" << agent_states_[i].x_pos;
  }
  ss << std::endl;
  return ss.str();
}
bool GridWorldState::EgoGoalReached() const {
  return agent_states_[ego_agent_idx].x_pos >=
         parameters_.ego_goal_reached_position;
}
void GridWorldState::ResetDepth() { depth_ = 0; }
const std::vector<AgentState> &GridWorldState::GetAgentStates() const {
  return agent_states_;
}
int GridWorldState::GetEgoPos() const {
  return agent_states_[ego_agent_idx].x_pos;
}
template <typename ActionType>
ActionType GridWorldState::GetLastAction(const AgentIdx &agent_idx) const {
  return static_cast<ActionType>(agent_states_[agent_idx].last_action);
}
std::shared_ptr<GridWorldState> GridWorldState::Clone() const {
  return std::make_shared<GridWorldState>(*this);
}
const GridWorldStateParameter &GridWorldState::GetParameters() const {
  return parameters_;
}
const RuleStateMap &GridWorldState::GetRuleStateMap() const {
  return rule_state_map_;
}
EvaluationMap GridWorldState::GetAgentLabels(AgentIdx agent_idx) const {
  EvaluationMap labels;
  std::vector<AgentState> next_other_agents(agent_states_);
  next_other_agents.erase(next_other_agents.begin() + agent_idx);
  // Create perspective from current agent
  World next_world(agent_states_[agent_idx], next_other_agents);

  for (const auto &le : label_evaluator_) {
    auto new_labels = le->evaluate(next_world);
    labels.insert(new_labels.begin(), new_labels.end());
  }
  return labels;
}
void GridWorldState::SetCollisionPositions(
    std::vector<AgentState> *agent_states) const {
  for (auto it_begin = agent_states->begin(); it_begin != agent_states->end();
       ++it_begin) {
    for (auto it = it_begin + 1; it != agent_states->end(); ++it) {
      if (check_collision(*it_begin, *it)) {
        int correct_pos = std::max(it_begin->x_pos, it->x_pos);
        it_begin->x_pos = correct_pos;
        it->x_pos = correct_pos;
        it_begin->lane = 0;
        it_begin->lane = 0;
        break;
      }
    }
  }
}
