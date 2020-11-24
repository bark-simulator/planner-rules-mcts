// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_BASE_TEST_ENV_H_
#define GRIDWORLD_BASE_TEST_ENV_H_

#include <deque>
#include <map>
#include <memory>
#include <vector>

#include "gridworld/common.hpp"
#include "gridworld/grid_world_state.hpp"
#include "gridworld/label_evaluator/evaluator_label_at_position.h"
#include "gridworld/label_evaluator/evaluator_label_collision.h"
#include "mvmcts/mvmcts.h"
#include "mvmcts/random_generator.h"
#include "mvmcts/statistics/slack_uct_statistic.h"

using namespace mvmcts;
using RuleMonitorSPtr = RuleMonitor::RuleMonitorSPtr;
typedef std::vector<Eigen::MatrixXi> History;

class BaseTestEnv {
 public:
  explicit BaseTestEnv(const ObjectiveVec &thres);
  ~BaseTestEnv() = default;
  static MvmctsParameters MakeMctsParameters(const ObjectiveVec& thres);
  static GridWorldStateParameter MakeGridWorldStateParameters();
  static std::vector<std::map<Rule, RuleMonitorSPtr>> MakeAutomata(
      size_t num_agents);
  static std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> MakeLabels(
      const GridWorldStateParameter &params);

  virtual JointAction Search(size_t num_iterations) = 0;
  virtual std::map<unsigned long, Eigen::VectorXd> GetEgoQval() = 0;

  const JointAction &GetJt() const;
  const std::deque<JointAction> &GetActionHistory() const;
  void SetJt(const JointAction &jt);

  const MvmctsParameters mcts_parameters;
  const GridWorldStateParameter grid_world_state_parameter;
  std::vector<std::shared_ptr<EvaluatorLabelBase<World>>> label_evaluators;
  std::vector<Reward> rewards;
  History state_history;
  std::shared_ptr<GridWorldState> state;

  static const int NUM_AGENTS = 3;
  static const char *ZIP_FORMULA;

 protected:
  RuleStateMap GetAutomataVec() const;

  std::vector<std::map<Rule, RuleMonitorSPtr>> automata_;

 private:
  JointAction jt_;
  std::deque<JointAction> action_history_;
};

#endif  // GRIDWORLD_BASE_TEST_ENV_H_
