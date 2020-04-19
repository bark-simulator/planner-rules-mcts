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
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/evaluation/evaluator_drivable_area.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::StateDefinition;
using ltl::Label;
using modules::world::LabelMap;
using modules::world::ObservedWorld;
using modules::world::ObservedWorldPtr;
using modules::world::evaluation::EvaluatorDrivableArea;

MvmctsStateMultiAgent::MvmctsStateMultiAgent(
    const modules::world::ObservedWorldPtr &observed_world,
    const MultiAgentRuleState &multi_agent_rule_state,
    const MvmctsStateParameters *params, const std::vector<AgentIdx> &agent_idx,
    unsigned int horizon)
    : multi_agent_rule_state_(multi_agent_rule_state),
      agent_idx_(agent_idx),
      state_params_(params),
      horizon_(horizon),
      observed_world_(observed_world),
      is_terminal_state_(false) {
  is_terminal_state_ = check_terminal();
}
std::shared_ptr<MvmctsStateMultiAgent> MvmctsStateMultiAgent::clone() const {
  return std::make_shared<MvmctsStateMultiAgent>(*this);
}
std::shared_ptr<MvmctsStateMultiAgent> MvmctsStateMultiAgent::execute(
    const mcts::JointAction &joint_action,
    std::vector<mcts::Reward> &rewards) const {
  EASY_FUNCTION();
  BARK_EXPECT_TRUE(!is_terminal());
  std::vector<AgentIdx> agent_ids = get_agent_idx();
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
      horizon_ - 1);

  AgentId world_agent_id;
  AgentPtr agent;
  for (size_t ai = 0; ai < num_agents; ++ai) {
    world_agent_id = agent_ids.at(ai);
    agent = predicted_world->GetAgent(world_agent_id);
    rewards[ai] = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
    if (agent && !agent->IsInactive()) {
      rewards[ai] += get_action_cost(agent);
      rewards[ai] += PotentialReward(
          world_agent_id, agent->GetCurrentState(),
          observed_world_->GetAgent(world_agent_id)->GetCurrentState());
      rewards[ai] += next_state->evaluate_rules(agent);
    } else if (observed_world_->GetAgent(world_agent_id) && !agent) {
      // Agent got out of map during this time step
      // So we obtain its out-of-map-penalty
      rewards[ai](0) += state_params_->OUT_OF_MAP_WEIGHT;
    }
  }
  return next_state;
}
mcts::ActionIdx MvmctsStateMultiAgent::get_num_actions(
    mcts::AgentIdx agent_idx) const {
  AgentId agent_id = get_agent_idx()[agent_idx];
  auto agent = observed_world_->GetAgent(agent_id);
  if (agent && !agent->IsInactive()) {
    auto agent_observed_world = std::make_shared<ObservedWorld>(std::move(observed_world_->Observe({agent_id})[0]));
    return std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
               agent->GetBehaviorModel())
        ->GetNumMotionPrimitives(agent_observed_world);
  } else {
    return 1;
  }
}
bool MvmctsStateMultiAgent::is_terminal() const { return is_terminal_state_; }
const std::vector<mcts::AgentIdx> MvmctsStateMultiAgent::get_agent_idx() const {
  return agent_idx_;
}
std::string MvmctsStateMultiAgent::sprintf() const { return std::string(); }

std::vector<Reward> MvmctsStateMultiAgent::get_final_reward() const {
  std::vector<AgentIdx> agent_ids = get_agent_idx();
  size_t num_agents = agent_ids.size();
  JointReward rewards(num_agents,
                      Reward::Zero(state_params_->REWARD_VECTOR_SIZE));
  for (size_t ai = 0; ai < num_agents; ++ai) {
    // Only get final reward for agents present
    if (observed_world_->GetAgent(agent_ids[ai])) {
      for (const auto &rule : multi_agent_rule_state_.at(agent_ids[ai])) {
        rewards[ai](rule.get_priority()) +=
            rule.get_automaton()->get_final_reward(rule);
      }
    }
  }
  return rewards;
}
Reward MvmctsStateMultiAgent::evaluate_rules(const AgentPtr &agent) {
  Reward reward = Reward::Zero(state_params_->REWARD_VECTOR_SIZE);
  // Create observed world from agent perspective
  ObservedWorld agent_observed_world =
      observed_world_->Observe({agent->GetAgentId()})[0];
  agent_observed_world.AddLabels(observed_world_->GetLabelEvaluators());

  // Get labels
  LabelMap label_map = agent_observed_world.EvaluateLabels();
  if (!is_terminal_state_ &&
      agent->GetAgentId() == observed_world_->GetEgoAgent()->GetAgentId()) {
    is_terminal_state_ =
        is_terminal_state_ || label_map.at(Label("collision_ego"));
  }
  if (!agent->IsInactive()) {
    for (auto &rule : multi_agent_rule_state_.at(agent->GetAgentId())) {
      reward(rule.get_priority()) +=
          rule.get_automaton()->evaluate(label_map, rule);
    }
    // Check if agent is in a terminal state
    if (label_map.at(Label("collision_ego"))) {
      observed_world_->GetAgent(agent->GetAgentId())->SetInactive();
    }
  }
  return reward;
}
JointReward MvmctsStateMultiAgent::evaluate_rules() {
  const auto agent_ids = get_agent_idx();
  JointReward rewards = JointReward(
      agent_ids.size(), Reward::Zero(state_params_->REWARD_VECTOR_SIZE));
  const auto agent_map = observed_world_->GetAgents();
  AgentId agent_id;
  for (size_t ai = 0; ai < agent_ids.size(); ++ai) {
    agent_id = agent_ids[ai];
    rewards[ai] = evaluate_rules(agent_map.at(agent_id));
  }
  return rewards;
}
const MultiAgentRuleState &MvmctsStateMultiAgent::get_multi_agent_rule_state()
    const {
  return multi_agent_rule_state_;
}

Eigen::VectorXf MvmctsStateMultiAgent::get_action_cost(
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
  //  LOG(INFO) << "Theta dot:"  << theta_dot << " Vel " << avg_vel << " Alat
  //  "
  //  << a_lat;
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

  //    Desired lane (squared)
  //  auto current_lane = agent->GetCurrentLane();
  //  if(current_lane) {
  //    const float lane_id_dev = agent->GetLocalMap()->GetGoalLaneId() ==
  //    current_lane->GetId() ? 0.0 : 1.0; reward(value_pos) +=
  //    state_params_->GOAL_LANE_DEVIATION_WEIGHT * lane_id_dev;
  //  }
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

bool MvmctsStateMultiAgent::check_terminal() const {
  bool terminal = false;
  auto ego = observed_world_->GetEgoAgent();
  if (!ego) {
    return true;
  }
  EvaluatorDrivableArea evaluator(observed_world_->GetEgoAgent()->GetAgentId());
  terminal = terminal || horizon_ == 0;
  terminal =
      terminal ||
      boost::get<bool>(evaluator.Evaluate(
          *std::dynamic_pointer_cast<const world::World>(observed_world_)));
  terminal = terminal || ego->AtGoal();
  return terminal;
}
const ObservedWorldPtr &MvmctsStateMultiAgent::GetObservedWorld() const {
  return observed_world_;
}
}  // namespace behavior
}  // namespace models
}  // namespace modules
