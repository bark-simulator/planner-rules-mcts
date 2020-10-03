// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <memory>
#include <string>
#include <vector>

#include "evaluation/evaluation.h"
#include "gridworld/test_runner.h"

using mvmcts::evaluation::QValWriter;

TestRunner::Result TestRunner::RunTest(size_t num_iter, int max_steps) {
  // Always recreate test environment to isolate test iterations
  int steps = 0;
  std::vector<Reward> step_reward(
      latest_test_env_->grid_world_state_parameter.num_other_agents + 1);
  // Store initial state in history
  latest_test_env_->state_history.emplace_back(GetStateVector().transpose());
  QValWriter qw(
      latest_test_env_->mcts_parameters.thres_uct_statistic_.THRESHOLD,
      q_val_fname_,
      latest_test_env_->grid_world_state_parameter.action_map.size());
  while (!latest_test_env_->state->IsTerminal() && steps < max_steps) {
    latest_test_env_->Search(num_iter);
    qw.WriteQVal(latest_test_env_->GetEgoQval(), latest_test_env_->GetJt()[0]);
    latest_test_env_->state = latest_test_env_->state->Execute(
        latest_test_env_->GetJt(), step_reward);
    latest_test_env_->state->ResetDepth();
    VLOG(1) << "Iteration: " << steps
            << ", Next state: " << latest_test_env_->state->PrintState()
            << ", Ego step reward: " << step_reward[0].transpose();
    PrintLabels();
    PrintRuleStates();
    latest_test_env_->rewards += step_reward;
    latest_test_env_->state_history.emplace_back(GetStateVector().transpose());
    ++steps;
  }
  latest_test_env_->rewards += latest_test_env_->state->GetTerminalReward();
  LOG(INFO) << "History:" << latest_test_env_->state_history;
  auto cumulated_ego_reward = latest_test_env_->rewards[0];
  Result r;
  r.collision = cumulated_ego_reward(0) < 0;
  r.violation = cumulated_ego_reward(1) < 0;
  r.pos = latest_test_env_->state_history.back()(0);
  r.value = cumulated_ego_reward(cumulated_ego_reward.size() - 1);
  return r;
}
Eigen::VectorXi TestRunner::GetStateVector() const {
  auto agent_states = latest_test_env_->state->GetAgentStates();
  Eigen::VectorXi state = Eigen::VectorXi::Zero(agent_states.size());
  for (size_t i = 0; i < agent_states.size(); ++i) {
    state(i) = agent_states[i].x_pos;
  }
  return state;
}

ostream& operator<<(ostream& os, const TestRunner::Result& result) {
  os << result.pos << "\t" << result.value << "\t"
     << TestRunner::Result::BoolToString(result.collision) << "\t"
     << TestRunner::Result::BoolToString(result.violation);
  return os;
}
ostream& TestRunner::Result::WriteHeader(ostream& os) {
  os << "Traveled distance\tBase reward\tCollision\tRule violation";
  return os;
}
void TestRunner::PrintLabels() {
  for (const auto& label : latest_test_env_->state->GetAgentLabels(0)) {
    VLOG(1) << label.first.GetLabelStr() << " : " << label.second;
  }
}
void TestRunner::PrintRuleStates() {
  for (const auto& rs : latest_test_env_->state->GetRuleStateMap()[0]) {
    VLOG(1) << rs.second;
  }
}
const std::shared_ptr<BaseTestEnv>& TestRunner::GetLatestTestEnv() const {
  return latest_test_env_;
}
void TestRunner::SetQValFname(const std::string& q_val_fname) {
  q_val_fname_ = q_val_fname;
}
