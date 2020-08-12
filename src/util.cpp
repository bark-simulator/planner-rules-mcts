// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "util.hpp"
#include "bark/models/behavior/rule_based/lane_change_behavior.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/dynamic/single_track.hpp"

#include <limits>
#include <vector>

namespace bark {
namespace models {
namespace behavior {

using mvmcts::ObjectiveVec;

mvmcts::MvmctsParameters MakeMctsParameters(const commons::ParamsPtr& params) {
  mvmcts::MvmctsParameters mcts_p;
  size_t reward_vec_size =
      params->GetInt("BehaviorMvmcts::RewardVectorSize",
                     "Number of dimensions of the reward vector", 1);
  mcts_p.COOP_FACTOR = params->GetReal("BehaviorMvmcts::CooperationFactor",
                                       "Cooperative behavior of agents", 0.0);
  mcts_p.DISCOUNT_FACTOR =
      params->GetReal("BehaviorMvmcts::DiscountFactor",
                      "Discount factor used in MDP problem", 0.9);
  mcts_p.REWARD_VEC_SIZE = reward_vec_size;

  // Disable for now. Partial rollouts due to computation time cause problems
  // with undecided liveness rules
  // TODO: Think about how this could be treated.

  //  mcts_p.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC =
  //  params->get_real(
  //          "BehaviorMvmcts::MaxSearchTimeRandomHeuristic",
  //          "Maximum time available for random rollout in milliseconds",
  //          1000)
  mcts_p.random_heuristic
      .MAX_SEARCH_TIME_RANDOM_HEURISTIC = std::numeric_limits<decltype(
      mcts_p.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC)>::infinity();
  mcts_p.random_heuristic.MAX_NUMBER_OF_ITERATIONS = params->GetInt(
      "BehaviorMvmcts::MaxNumIterationsRandomHeuristic",
      "Maximum number of environment steps performed by random heuristic", 30);

  mcts_p.uct_statistic.EXPLORATION_CONSTANT = StdToEigen(
          params->GetListFloat("BehaviorMvmcts::UCTExplorationConstant",
                               "Exploration constant of UCT",
                               std::vector<float>(reward_vec_size, 1.42f)))
          .cast<double>();
  assert(mcts_p.uct_statistic.EXPLORATION_CONSTANT.size() == reward_vec_size);
  std::vector<float> default_lower_bound(reward_vec_size, -1.0);
  default_lower_bound[reward_vec_size - 1] = -1000.0;
  auto lower_bound = params->GetListFloat(
      "BehaviorMvmcts::ReturnLowerBound",
      "Lower return bound used for normalization in UCT Statistic",
    default_lower_bound);
  assert(lower_bound.size() == reward_vec_size);
  mcts_p.uct_statistic.LOWER_BOUND = StdToEigen(lower_bound);

  mcts_p.uct_statistic.UPPER_BOUND = StdToEigen(params->GetListFloat(
      "BehaviorMvmcts::ReturnUpperBound",
      "Upper return bound used for normalization in UCT Statistic", std::vector<float>(reward_vec_size, 0.0)));
  assert(mcts_p.uct_statistic.UPPER_BOUND.size() == reward_vec_size);

  mcts_p.thres_uct_statistic_.THRESHOLD = StdToEigen(params->GetListFloat(
      "BehaviorMvmcts::Threshold", "Thresholds for improving lower level goals", std::vector<float>(mcts_p.REWARD_VEC_SIZE, std::numeric_limits<float>::max())));
  assert(mcts_p.thres_uct_statistic_.THRESHOLD.size() == reward_vec_size);

  mcts_p.thres_uct_statistic_.EPSILON = params->GetReal("BehaviorMvmcts::EpsilonGreedy",
                      "Probability that a random action is chosen", 0.1);
  mcts_p.thres_greedy_statistic_.DECAY1 = params->GetReal(
      "BehaviorMvmcts::Decay1", "Counter of the decay term", .8);
  mcts_p.thres_greedy_statistic_.DECAY2 =
      params->GetReal("BehaviorMvmcts::Decay2",
                      "Square root of the denominator of the decay term", .12);
  return mcts_p;
}

Eigen::VectorXf StdToEigen(const std::vector<float>& v) {
  Eigen::VectorXf e = Eigen::VectorXf::Zero(v.size());
  for(size_t i = 0; i < v.size(); ++i) {
    e(i) = v[i];
  }
  return e;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
