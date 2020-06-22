// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "mvmcts_state_multi_agent.hpp"

#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <easy/profiler.h>

#include "ltl/label.h"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::StateDefinition;
using ltl::Label;
using modules::world::evaluation::LabelMap;
using modules::world::ObservedWorld;
using modules::world::ObservedWorldPtr;
using modules::world::evaluation::EvaluatorDrivableArea;
using modules::world::evaluation::EvaluatorCollisionEgoAgent;
using modules::models::execution::ExecutionStatus;

MvmctsStateMultiAgent::MvmctsStateMultiAgent(
    const modules::world::ObservedWorldPtr &observed_world,
    const MultiAgentRuleState &multi_agent_rule_state,
    const MvmctsStateParameters *params, const std::vector<AgentIdx> &agent_idx,
    unsigned int horizon, const LabelEvaluators* label_evaluators)
    : multi_agent_rule_state_(multi_agent_rule_state),
      agent_idx_(agent_idx),
      state_params_(params),
      horizon_(horizon),
      observed_world_(observed_world),
      is_terminal_state_(false),
      label_evaluators_(label_evaluators){
  is_terminal_state_ = CheckTerminal();
}
std::shared_ptr<MvmctsStateMultiAgent> MvmctsStateMultiAgent::Clone() const {
  return std::make_shared<MvmctsStateMultiAgent>(*this);
}
std::shared_ptr<MvmctsStateMultiAgent> MvmctsStateMultiAgent::Execute(
    const mcts::JointAction &joint_action,
    std::vector<mcts::Reward> &rewards) const {
  EASY_FUNCTION();
  BARK_EXPECT_TRUE(!IsTerminal());
  std::vector<AgentIdx> agent_ids = GetAgentIdx();
  size_t num_agents = agent_ids.size();
  rewards.resize(num_agents, Reward::Zero(state_params_->REWARD_VECTOR_SIZE));

  // Map actions from mcts AgentIdx to Bark AgentId
  std::unordered_map<AgentId, DiscreteAction> agent_action_map;
  for (size_t i = 0; i < num_agents; ++i) {
    agent_action_map.insert({agent_ids[i], DiscreteAction(joint_action[i])});
  }
  auto predicted_world =
      std::dynamic_pointer_cast<ObservedWorld>(observed_world_->Predict(
          state_params_->PREDICTION_TIME_SPAN, agent_action_map));
  auto next_state = std::make_shared<MvmctsStateMultiAgent>(
      predicted_world, multi_agent_rule_state_, state_params_, agent_idx_,
      horizon_ - 1, label_evaluators_);

  AgentId world_agent_id;
  AgentPtr agent;
  for (size_t ai = 0; ai < num_agents; ++ai) {
    world_agent_id = agent_ids.at(ai);
    agent = predicted_world->GetAgent(world_agent_id);
    rewards[ai] = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
    if (agent && agent->GetExecutionStatus() == ExecutionStatus::VALID) {
      rewards[ai] += GetActionCost(agent);
      rewards[ai] += PotentialReward(
          world_agent_id, agent->GetCurrentState(),
          observed_world_->GetAgent(world_agent_id)->GetCurrentState());
      rewards[ai] += next_state->EvaluateRules(agent);
    } else if (observed_world_->GetAgent(world_agent_id) && !agent) {
      // Agent got out of map during this time step
      // So we obtain its out-of-map-penalty
      rewards[ai](0) += state_params_->OUT_OF_MAP_WEIGHT;
    }
  }
  return next_state;
}
mcts::ActionIdx MvmctsStateMultiAgent::GetNumActions(
    mcts::AgentIdx agent_idx) const {
  AgentId agent_id = GetAgentIdx()[agent_idx];
  auto agent = observed_world_->GetAgent(agent_id);
  if (agent && agent->GetExecutionStatus() == ExecutionStatus::VALID) {
    auto agent_observed_world = std::make_shared<ObservedWorld>(std::move(observed_world_->Observe({agent_id})[0]));
    return std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
               agent->GetBehaviorModel())
        ->GetNumMotionPrimitives(agent_observed_world);
  } else {
    return 1;
  }
}
bool MvmctsStateMultiAgent::IsTerminal() const { return is_terminal_state_; }
const std::vector<mcts::AgentIdx> MvmctsStateMultiAgent::GetAgentIdx() const {
  return agent_idx_;
}
std::string MvmctsStateMultiAgent::PrintState() const { return std::string(); }

std::vector<Reward> MvmctsStateMultiAgent::GetTerminalReward() const {
  std::vector<AgentIdx> agent_ids = GetAgentIdx();
  size_t num_agents = agent_ids.size();
  JointReward rewards(num_agents,
                      Reward::Zero(state_params_->REWARD_VECTOR_SIZE));
  for (size_t ai = 0; ai < num_agents; ++ai) {
    // Only get final reward for agents present
    if (observed_world_->GetAgent(agent_ids[ai])) {
      for (const auto &rule : multi_agent_rule_state_.at(agent_ids[ai])) {
        rewards[ai](rule.GetPriority()) +=
            rule.GetAutomaton()->FinalTransit(rule);
      }
    }
  }
  return rewards;
}
Reward MvmctsStateMultiAgent::EvaluateRules(const AgentPtr &agent) {
  Reward reward = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
  // Create observed world from agent perspective
  ObservedWorld agent_observed_world =
      observed_world_->Observe({agent->GetAgentId()})[0];

  // Obtain labels
  LabelMap label_map;
  for(const auto& l : *label_evaluators_) {
      auto labels = l->Evaluate(agent_observed_world);
      label_map.insert(labels.begin(), labels.end());
  }

  if (agent->GetExecutionStatus() == ExecutionStatus::VALID) {
    for (auto &rule : multi_agent_rule_state_.at(agent->GetAgentId())) {
      reward(rule.GetPriority()) +=
          rule.GetAutomaton()->Evaluate(label_map, rule);
    }
    // Check if agent has collided
    if (label_map.at(Label("collision_ego"))) {
        // TODO: Maybe use a separate value for collisions?
      observed_world_->GetAgent(agent->GetAgentId())->GetExecutionModel()->SetExecutionStatus(ExecutionStatus::INVALID);
    }
  }
  return reward;
}
JointReward MvmctsStateMultiAgent::EvaluateRules() {
  const auto agent_ids = GetAgentIdx();
  JointReward rewards = JointReward(
      agent_ids.size(), Reward::Zero(state_params_->REWARD_VECTOR_SIZE));
  const auto agent_map = observed_world_->GetAgents();
  AgentId agent_id;
  for (size_t ai = 0; ai < agent_ids.size(); ++ai) {
    agent_id = agent_ids[ai];
    rewards[ai] = EvaluateRules(agent_map.at(agent_id));
  }
  return rewards;
}
const MultiAgentRuleState &MvmctsStateMultiAgent::GetMultiAgentRuleState()
    const {
  return multi_agent_rule_state_;
}

Eigen::VectorXf MvmctsStateMultiAgent::GetActionCost(
    const std::shared_ptr<const world::Agent> &agent) const {
  Eigen::VectorXf reward = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
  const size_t value_pos = state_params_->REWARD_VECTOR_SIZE - 1;
  auto traj = agent->GetBehaviorModel()->GetLastTrajectory();
  const size_t traj_size = traj.rows();
  const float dt = state_params_->PREDICTION_TIME_SPAN;
  const float a = (traj(traj_size - 1, dynamic::VEL_POSITION) -
                   traj(0, dynamic::VEL_POSITION)) /
                  dt;
  reward(value_pos) +=
      state_params_->ACCELERATION_WEIGHT * a * a * dt;  //  Acceleration

  const float theta_dot = (traj(traj_size - 1, dynamic::THETA_POSITION) -
                           traj(0, dynamic::THETA_POSITION)) /
                          dt;
  const float avg_vel = 0.5f * a * dt + traj(0, dynamic::VEL_POSITION);
  const float a_lat = theta_dot * avg_vel;  //  Radial acceleration

  reward(value_pos) +=
      state_params_->RADIAL_ACCELERATION_WEIGHT * a_lat * a_lat * dt;

  //  Desired velocity
  reward(value_pos) += state_params_->DESIRED_VELOCITY_WEIGHT *
                       fabs(avg_vel - state_params_->DESIRED_VELOCITY) * dt;
  //  Lane center deviation
  const auto &lane_corridor = agent->GetRoadCorridor()->GetCurrentLaneCorridor(
      agent->GetCurrentPosition());
  if (lane_corridor) {
    const float lane_center_dev =
        commons::transformation::FrenetPosition(agent->GetCurrentPosition(),
                                                lane_corridor->GetCenterLine())
            .lat;
    reward(value_pos) +=
        state_params_->LANE_CENTER_WEIGHT * fabs(lane_center_dev) * dt;
  }
  return reward;
}

Reward MvmctsStateMultiAgent::PotentialReward(AgentId agent_id, const State &new_state,
                                    const State &current_state) const {
  Eigen::VectorXf reward = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
  reward(reward.size() - 1) =
      state_params_->DISCOUNT_FACTOR * Potential(agent_id, new_state) -
      Potential(agent_id, current_state);
  return reward;
}
inline float MvmctsStateMultiAgent::Potential(
    AgentId agent_id, const modules::models::dynamic::State &state) const {
  const float dv = std::abs(state(dynamic::StateDefinition::VEL_POSITION) -
                            state_params_->DESIRED_VELOCITY);
  return -state_params_->POTENTIAL_WEIGHT *
         state_params_->PREDICTION_TIME_SPAN * dv;
}

bool MvmctsStateMultiAgent::CheckTerminal() const {
  bool terminal = false;
  auto ego = observed_world_->GetEgoAgent();
  if (!ego) {
    return true;
  }
  EvaluatorDrivableArea evaluator_out_of_map;
  EvaluatorCollisionEgoAgent evaluator_collision;
  // Planning horizon reached
  terminal = terminal || horizon_ == 0;
  // Out of map
  terminal =
      terminal ||
      boost::get<bool>(evaluator_out_of_map.Evaluate(
          *std::dynamic_pointer_cast<const world::World>(observed_world_)));
  // Collision
  terminal = terminal || boost::get<bool>(evaluator_collision.Evaluate(
        *std::dynamic_pointer_cast<const world::World>(observed_world_)));
  // Goal reached
  terminal = terminal || ego->AtGoal();
  return terminal;
}
const ObservedWorldPtr &MvmctsStateMultiAgent::GetObservedWorld() const {
  return observed_world_;
}
}  // namespace behavior
}  // namespace models
}  // namespace modules
