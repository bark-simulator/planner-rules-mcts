// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "util.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/mobil/mobil.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/dynamic/single_track.hpp"

#include <limits>
#include <vector>

namespace modules {
namespace models {
namespace behavior {

using mcts::ObjectiveVec;

mcts::MctsParameters MakeMctsParameters(const commons::ParamsPtr &params) {
  mcts::MctsParameters mcts_p;
  size_t reward_vec_size =
      params->GetInt("BehaviorMCTSAgent::RewardVectorSize",
                      "Number of dimensions of the reward vector", 1);
  mcts_p.COOP_FACTOR =
      params->GetReal("BehaviorMCTSAgent::CooperationFactor",
                       "Cooperative behavior of agents", 0.0);
  mcts_p.DISCOUNT_FACTOR =
      params->GetReal("BehaviorMCTSAgent::DiscountFactor",
                       "Discount factor used in MDP problem", 0.9);
  mcts_p.REWARD_VEC_SIZE = reward_vec_size;

  // Disable for now. Partial rollouts due to computation time cause problems
  // with undecided liveness rules
  // TODO: Think about how this could be treated.

  //  mcts_p.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC =
  //  params->get_real(
  //          "BehaviorMCTSAgent::MaxSearchTimeRandomHeuristic",
  //          "Maximum time available for random rollout in milliseconds",
  //          1000)
  mcts_p.random_heuristic
      .MAX_SEARCH_TIME_RANDOM_HEURISTIC = std::numeric_limits<decltype(
      mcts_p.random_heuristic.MAX_SEARCH_TIME_RANDOM_HEURISTIC)>::infinity();
  mcts_p.random_heuristic.MAX_NUMBER_OF_ITERATIONS = params->GetInt(
      "BehaviorMCTSAgent::MaxNumIterationsRandomHeuristic",
      "Maximum number of environment steps performed by random heuristic", 100);

  mcts_p.uct_statistic.EXPLORATION_CONSTANT = StdToEigen(
      params->GetListFloat("BehaviorMCTSAgent::UCTExplorationConstant",
                       "Exploration constant of UCT", std::vector<float>(reward_vec_size, 0.7f))).cast<double>();
  assert(mcts_p.uct_statistic.EXPLORATION_CONSTANT.size() == reward_vec_size);
  mcts_p.uct_statistic.PROGRESSIVE_WIDENING_ENABLED =
      params->GetBool("BehaviorMCTSAgent::EnableProgressiveWidening",
                       "Enables the Progressive Widening heursitc", false);
  mcts_p.uct_statistic.PROGRESSIVE_WIDENING_ALPHA =
      params->GetReal("BehaviorMCTSAgent::ProgressiveWideningAlpha",
                       "Parameter for ProgressiveWidening", 0.5);
  std::vector<float> default_lower_bound(reward_vec_size, -1.0);
  default_lower_bound[reward_vec_size - 1] = -50000.0;
  auto lower_bound = params->GetListFloat(
      "BehaviorMCTSAgent::ReturnLowerBound",
      "Lower return bound used for normalization in UCT Statistic",
    default_lower_bound);
  assert(lower_bound.size() == reward_vec_size);
  mcts_p.uct_statistic.LOWER_BOUND = StdToEigen(lower_bound);

  mcts_p.uct_statistic.UPPER_BOUND = StdToEigen(params->GetListFloat(
      "BehaviorMCTSAgent::ReturnUpperBound",
  "Upper return bound used for normalization in UCT Statistic", std::vector<float>(reward_vec_size, 0.0)));
  assert(mcts_p.uct_statistic.UPPER_BOUND.size() == reward_vec_size);

  mcts_p.thres_uct_statistic_.THRESHOLD = StdToEigen(params->GetListFloat(
      "BehaviorMCTSAgent::Threshold",
      "Thresholds for improving lower level goals", std::vector<float>(mcts_p.REWARD_VEC_SIZE, std::numeric_limits<float>::max())));
  assert(mcts_p.thres_uct_statistic_.THRESHOLD.size() == reward_vec_size);

  mcts_p.thres_uct_statistic_.EPSILON = params->GetReal(
      "BehaviorMCTSAgent::EpsilonGreedy",
      "Probability that a random action is chosen", 0.1);
  mcts_p.e_greedy_uct_statistic_.EPSILON = params->GetReal(
      "BehaviorMCTSAgent::ExplorationProb", "Initial probability of exploration", 1.0);
  mcts_p.e_greedy_uct_statistic_.MINIMUM_EPSILON = params->GetReal(
      "BehaviorMCTSAgent::MinEpsilon", "Minimum exploration probability", 0.1);
  mcts_p.e_greedy_uct_statistic_.EPSILON_DECAY = params->GetReal(
      "BehaviorMCTSAgent::EpsilonDecay", "Exponential discount factor for epsilon", 0.9997);
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
}  // namespace modules
