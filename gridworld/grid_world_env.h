// Copyright (c) 2020 Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef GRIDWORLD_CROSSING_TEST_ENV_H_
#define GRIDWORLD_CROSSING_TEST_ENV_H_

#include <limits>
#include <map>
#include <utility>

#include "gridworld/base_test_env.h"
#include "mcts/heuristics/random_heuristic.h"
#include "mcts/mcts.h"
#include "mcts/statistics/uct_statistic.h"

template <class Stats = UctStatistic<>, class Heuristic = RandomHeuristic>
class CrossingTestEnv : public BaseTestEnv {
 public:
  explicit CrossingTestEnv(const ObjectiveVec& thres)
      : BaseTestEnv(thres), mcts(mcts_parameters) {}
  JointAction Search(size_t num_iterations) override {
    mcts.Search(*(this->state), std::numeric_limits<unsigned int>::max(),
                num_iterations);
    this->SetJt(mcts.ReturnBestAction());
    VLOG(1) << "Best action: " << this->GetJt();
    return this->GetJt();
  }
  std::map<unsigned long, Eigen::VectorXf> GetEgoQval() override {
    return mcts.GetRoot()->GetEgoIntNode().GetExpectedRewards();
  }
  Mcts<GridWorldState, Stats, Stats, Heuristic> mcts;
};

#endif  // GRIDWORLD_CROSSING_TEST_ENV_H_
