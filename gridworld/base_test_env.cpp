// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gridworld/base_test_env.h"

#include <limits>

#include "gridworld/label_evaluator/evaluator_label_ego_range.h"
#include "gridworld/label_evaluator/evaluator_label_in_direct_front.h"
#include "gridworld/label_evaluator/evaluator_label_in_range.h"

const char *BaseTestEnv::ZIP_FORMULA =
    "(in_direct_front_x#0 & !merged_e & (in_direct_front_x#0 | merged_x#0) U "
    "merged_e) -> G(merged_e & merged_x#0 "
    "-> !in_direct_front_x#0)";

BaseTestEnv::BaseTestEnv(const ObjectiveVec &thres)
    : mcts_parameters(MakeMctsParameters(thres)),
      grid_world_state_parameter(MakeGridWorldStateParameters()),
      label_evaluators(MakeLabels(grid_world_state_parameter)),
      rewards(grid_world_state_parameter.num_other_agents + 1,
              Reward::Zero(mcts_parameters.REWARD_VEC_SIZE)),
      automata_(MakeAutomata(NUM_AGENTS)),
      jt_(NUM_AGENTS, static_cast<int>(Actions::FORWARD)) {
  RuleStateMap aut_v = GetAutomataVec();
  std::vector<AgentState> agent_states(NUM_AGENTS);
  agent_states[0].id = 0;
  agent_states[1].id = 1;
  agent_states[2].id = 2;
  agent_states[0].x_pos = grid_world_state_parameter.merging_point - 6;
  agent_states[1].x_pos = grid_world_state_parameter.merging_point - 2;
  agent_states[2].x_pos = grid_world_state_parameter.merging_point - 8;
  // Set agents 0 and 1 to the same lane
  agent_states[1].lane = agent_states[0].lane;
  agent_states[1].init_lane = agent_states[0].init_lane;

  state = std::make_shared<GridWorldState>(
      agent_states, false, aut_v, label_evaluators, grid_world_state_parameter,
      0, std::vector<bool>(agent_states.size(), false));
}
MvmctsParameters BaseTestEnv::MakeMctsParameters(const ObjectiveVec& thres) {
  MvmctsParameters param;

  param.REWARD_VEC_SIZE = 3;

  param.random_heuristic.MAX_NUMBER_OF_ITERATIONS = 30;

  param.uct_statistic.EXPLORATION_CONSTANT =
      Eigen::VectorXd::Zero(param.REWARD_VEC_SIZE);
  param.uct_statistic.EXPLORATION_CONSTANT << 0.8, 0.8, 0.8;

  param.uct_statistic.LOWER_BOUND = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);
  param.uct_statistic.UPPER_BOUND = ObjectiveVec::Zero(param.REWARD_VEC_SIZE);

  param.thres_uct_statistic_.EPSILON = 1.0;

  param.slack_uct_statistic_.SLACK_FACTOR = 0.2;

  assert(thres.size() == param.REWARD_VEC_SIZE);
  param.thres_uct_statistic_.THRESHOLD = thres;
  param.thres_uct_statistic_.EPSILON = 0.0;

  param.thres_greedy_statistic_.DECAY1 = .8;
  param.thres_greedy_statistic_.DECAY2 = .12;

  param.uct_statistic.LOWER_BOUND << -1.0f, -1.0f, -100.0f;
  param.uct_statistic.UPPER_BOUND << 0.0f, 0.0f, 0.0f;
  param.COOP_FACTOR = 0.0;
  param.DISCOUNT_FACTOR = 0.95;
  param.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC =
      std::numeric_limits<double>::infinity();
  return param;
}
GridWorldStateParameter BaseTestEnv::MakeGridWorldStateParameters() {
  GridWorldStateParameter p;
  p.reward_vec_size = 3;

  p.potential_weight = 0.0f;
  p.action_map = {static_cast<int>(Actions::FORWARD),
                  static_cast<int>(Actions::WAIT),
                  static_cast<int>(Actions::BACKWARD)};

  p.num_other_agents = 2;
  p.ego_goal_reached_position = 1000;
  p.state_x_length = 1000;
  p.merging_point = 8;
  p.terminal_depth_ = 8;
  p.acceleration_weight = 1.5f;
  p.speed_deviation_weight = 5.0f;

  return p;
}
std::vector<std::map<Rule, RuleMonitorSPtr>> BaseTestEnv::MakeAutomata(
    size_t num_agents) {
  std::vector<std::map<Rule, RuleMonitorSPtr>> automata(num_agents);
  automata[0].insert(
      {Rule::NO_COLLISION, RuleMonitor::MakeRule("G !collision", -1.0f, 0)});
  automata[0].insert({Rule::ZIP, RuleMonitor::MakeRule(ZIP_FORMULA, -1, 1)});
  // Same default rules for all agents
  for (size_t i = 1; i < automata.size(); ++i) {
    automata[i] = automata[0];
  }
  return automata;
}
std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> BaseTestEnv::MakeLabels(
    const GridWorldStateParameter &params) {
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> labels;
  labels.emplace_back(std::make_shared<EvaluatorLabelCollision>(
      "collision", params.merging_point));
  labels.emplace_back(std::make_shared<EvaluatorLabelEgoRange>(
      "merged_e", params.merging_point, params.state_x_length));
  labels.emplace_back(std::make_shared<EvaluatorLabelInRange>(
      "merged_x", params.merging_point, params.state_x_length));
  labels.emplace_back(
      std::make_shared<EvaluatorLabelInDirectFront>("in_direct_front_x"));
  return labels;
}
RuleStateMap BaseTestEnv::GetAutomataVec() const {
  RuleStateMap aut_v(automata_.size());
  std::vector<int> agent_ids(automata_.size(), 0);
  std::iota(agent_ids.begin(), agent_ids.end(), 0);
  std::vector<int> others;
  for (size_t i = 0; i < automata_.size(); ++i) {
    VLOG(1) << "Rules for agent " << i << ":";
    others = agent_ids;
    others.erase(others.begin() + i);
    for (const auto &it : automata_[i]) {
      VLOG(1) << *(it.second);
      auto rule_states = it.second->MakeRuleState(others, {});
      for (const auto &rs : rule_states) {
        aut_v[i].insert({it.first, rs});
      }
    }
  }
  return aut_v;
}
void BaseTestEnv::SetJt(const JointAction &jt) {
  jt_ = jt;
  action_history_.emplace_back(jt);
}
const std::deque<JointAction> &BaseTestEnv::GetActionHistory() const {
  return action_history_;
}
const JointAction &BaseTestEnv::GetJt() const { return jt_; }
