// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/mvmcts_state.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/ltl/label/label.h"
#include "bark/world/objects/agent.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::execution::ExecutionStatus;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::evaluation::EvaluatorDrivableArea;
using bark::world::evaluation::LabelMap;
using dynamic::StateDefinition;
using ltl::Label;

MvmctsState::MvmctsState(const bark::world::ObservedWorldPtr& observed_world,
                         const MultiAgentRuleState& multi_agent_rule_state,
                         const MvmctsStateParameters* params,
                         const std::vector<AgentIdx>& agent_idx,
                         unsigned int horizon,
                         const LabelEvaluators* label_evaluators)
    : multi_agent_rule_state_(multi_agent_rule_state),
      agent_idx_(agent_idx),
      state_params_(params),
      horizon_(horizon),
      observed_world_(observed_world),
      is_terminal_state_(false),
      label_evaluators_(label_evaluators) {
  is_terminal_state_ = CheckTerminal();
}
std::shared_ptr<MvmctsState> MvmctsState::Clone() const {
  return std::make_shared<MvmctsState>(*this);
}
std::shared_ptr<MvmctsState> MvmctsState::Execute(
    const JointAction& joint_action, std::vector<Reward>& rewards) const {
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
  auto next_state = std::make_shared<MvmctsState>(
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
      if (agent->AtGoal()) {
        rewards[ai](state_params_->REWARD_VECTOR_SIZE - 1) +=
            state_params_->GOAL_REWARD;
      }
    } else if (observed_world_->GetAgent(world_agent_id) && !agent) {
      // Agent got out of map during this time step
      // So we obtain its out-of-map-penalty
      rewards[ai](0) += state_params_->OUT_OF_MAP_WEIGHT;
    }
  }
  return next_state;
}
ActionIdx MvmctsState::GetNumActions(AgentIdx agent_idx) const {
  AgentId agent_id = GetAgentIdx()[agent_idx];
  auto agent = observed_world_->GetAgent(agent_id);
  if (agent && agent->GetExecutionStatus() == ExecutionStatus::VALID) {
    auto agent_observed_world = std::make_shared<ObservedWorld>(
        std::move(observed_world_->Observe({agent_id})[0]));
    return std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
               agent->GetBehaviorModel())
        ->GetNumMotionPrimitives(agent_observed_world);
  } else {
    return 1;
  }
}
bool MvmctsState::IsTerminal() const { return is_terminal_state_; }
const std::vector<AgentIdx> MvmctsState::GetAgentIdx() const {
  return agent_idx_;
}
std::string MvmctsState::PrintState() const { return std::string(); }

std::vector<Reward> MvmctsState::GetTerminalReward() const {
  std::vector<AgentIdx> agent_ids = GetAgentIdx();
  size_t num_agents = agent_ids.size();
  JointReward rewards(num_agents,
                      Reward::Zero(state_params_->REWARD_VECTOR_SIZE));
  for (size_t ai = 0; ai < num_agents; ++ai) {
    // Only get final reward for agents present
    auto agent = observed_world_->GetAgent(agent_ids[ai]);
    if (agent) {
      for (const auto& rule : multi_agent_rule_state_.at(agent_ids[ai])) {
        rewards[ai](rule.GetPriority()) +=
            rule.GetAutomaton()->FinalTransit(rule);
      }
    }
  }
  return rewards;
}
Reward MvmctsState::EvaluateRules(const AgentPtr& agent) {
  Reward reward = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
  // Create observed world from agent perspective
  ObservedWorld agent_observed_world =
      observed_world_->Observe({agent->GetAgentId()})[0];

  // Obtain labels
  LabelMap label_map;
  for (const auto& l : *label_evaluators_) {
    auto labels = l->Evaluate(agent_observed_world);
    label_map.insert(labels.begin(), labels.end());
  }

  if (agent->GetExecutionStatus() == ExecutionStatus::VALID) {
    for (auto& rule : multi_agent_rule_state_.at(agent->GetAgentId())) {
      reward(rule.GetPriority()) +=
          rule.GetAutomaton()->Evaluate(label_map, rule);
    }
    // Check if agent has collided
    if (label_map.at(Label("collision_ego"))) {
      // TODO: Maybe use a separate value for collisions?
      observed_world_->GetAgent(agent->GetAgentId())
          ->GetExecutionModel()
          ->SetExecutionStatus(ExecutionStatus::INVALID);
    }
  }
  return reward;
}
JointReward MvmctsState::EvaluateRules() {
  const auto agent_ids = GetAgentIdx();
  JointReward rewards = JointReward(
      agent_ids.size(), Reward::Zero(state_params_->REWARD_VECTOR_SIZE));
  const auto agent_map = observed_world_->GetValidAgents();
  AgentId agent_id;
  for (size_t ai = 0; ai < agent_ids.size(); ++ai) {
    agent_id = agent_ids[ai];
    rewards[ai] = EvaluateRules(agent_map.at(agent_id));
  }
  return rewards;
}
const MultiAgentRuleState& MvmctsState::GetMultiAgentRuleState() const {
  return multi_agent_rule_state_;
}

Eigen::VectorXf MvmctsState::GetActionCost(
    const std::shared_ptr<const world::Agent>& agent) const {
  float action_cost = 0.0f;
  Eigen::VectorXf reward = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
  const size_t value_pos = state_params_->REWARD_VECTOR_SIZE - 1;
  auto traj = agent->GetBehaviorModel()->GetLastTrajectory();
  const size_t traj_size = traj.rows();
  const float dt = state_params_->PREDICTION_TIME_SPAN;
  const float a = (traj(traj_size - 1, dynamic::VEL_POSITION) -
                   traj(0, dynamic::VEL_POSITION)) /
                  dt;
  action_cost +=
      state_params_->ACCELERATION_WEIGHT * a * a * dt;  //  Acceleration

  const float theta_dot = (traj(traj_size - 1, dynamic::THETA_POSITION) -
                           traj(0, dynamic::THETA_POSITION)) /
                          dt;
  const float avg_vel = 0.5f * a * dt + traj(0, dynamic::VEL_POSITION);
  const float a_lat = theta_dot * avg_vel;  //  Radial acceleration

  action_cost += state_params_->RADIAL_ACCELERATION_WEIGHT * a_lat * a_lat * dt;

  //  Desired velocity
  action_cost += state_params_->DESIRED_VELOCITY_WEIGHT *
                 fabs(avg_vel - state_params_->DESIRED_VELOCITY) * dt;
  //  Lane center deviation
  const auto& lane_corridor = agent->GetRoadCorridor()->GetCurrentLaneCorridor(
      agent->GetCurrentPosition());
  if (lane_corridor) {
    const float lane_center_dev =
        commons::transformation::FrenetPosition(agent->GetCurrentPosition(),
                                                lane_corridor->GetCenterLine())
            .lat;
    action_cost +=
        state_params_->LANE_CENTER_WEIGHT * fabs(lane_center_dev) * dt;
  }
  reward(value_pos) = action_cost;
  return reward;
}

Reward MvmctsState::PotentialReward(AgentId agent_id, const State& new_state,
                                    const State& current_state) const {
  Eigen::VectorXf reward = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
  reward(reward.size() - 1) =
      state_params_->DISCOUNT_FACTOR * Potential(agent_id, new_state) -
      Potential(agent_id, current_state);
  return reward;
}
inline float MvmctsState::Potential(
    AgentId agent_id, const bark::models::dynamic::State& state) const {
  const float dv = std::abs(state(dynamic::StateDefinition::VEL_POSITION) -
                            state_params_->DESIRED_VELOCITY);
  return -state_params_->POTENTIAL_WEIGHT *
         state_params_->PREDICTION_TIME_SPAN * dv;
}

bool MvmctsState::CheckTerminal() const {
  auto ego = observed_world_->GetEgoAgent();
  if (!ego) {
    return true;
  }
  EvaluatorDrivableArea evaluator_out_of_map;
  EvaluatorCollisionEgoAgent evaluator_collision;
  // Planning horizon reached
  const bool horizon_reached = (horizon_ == 0);
  // Out of map
  const bool out_of_map =
      boost::get<bool>(evaluator_out_of_map.Evaluate(*observed_world_));
  // Collision
  const bool collision =
      boost::get<bool>(evaluator_collision.Evaluate(*observed_world_));
  // Goal reached
  const bool at_goal = ego->AtGoal();
  const bool terminal = horizon_reached || out_of_map || collision || at_goal;
  return terminal;
}
const ObservedWorldPtr& MvmctsState::GetObservedWorld() const {
  return observed_world_;
}
}  // namespace behavior
}  // namespace models
}  // namespace bark
