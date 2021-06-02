// Copyright (c) 2020 fortiss GmbH
//
// Authors: Klemens Esterle, Luis Gressenbuch
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_PLANNER_RULES_MCTS_RULES_MCTS_STATE_PARAMETERS_HPP_
#define MODULES_MODELS_BEHAVIOR_PLANNER_RULES_MCTS_RULES_MCTS_STATE_PARAMETERS_HPP_

#include "bark/commons/params/setter_params.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::commons::Params;

class RulesMctsStateParameters {
 public:
  RulesMctsStateParameters(
      const double collision_weight, const double out_of_map_weight,
      const double potential_weight, const double acceleration_weight,
      const double radial_acceleration_weight,
      const double desired_velocity_weight, const double lane_center_weight,
      const unsigned int reward_vector_size, const double prediction_time_span,
      const double desired_velocity, const unsigned int horizon,
      const double discount_factor, const double goal_reward,
      const bool use_rule_reward_for_ego_only);
  explicit RulesMctsStateParameters(const commons::ParamsPtr& params);
  const double COLLISION_WEIGHT;
  const double OUT_OF_MAP_WEIGHT;
  const double POTENTIAL_WEIGHT;
  const double ACCELERATION_WEIGHT;
  const double RADIAL_ACCELERATION_WEIGHT;
  const double DESIRED_VELOCITY_WEIGHT;
  const double LANE_CENTER_WEIGHT;
  const unsigned int REWARD_VECTOR_SIZE;
  const double PREDICTION_TIME_SPAN;
  const double DESIRED_VELOCITY;
  const unsigned int HORIZON;
  const double DISCOUNT_FACTOR;
  const double GOAL_REWARD;
  const bool USE_RULE_REWARD_FOR_EGO_ONLY;
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // MODULES_MODELS_BEHAVIOR_PLANNER_RULES_MCTS_RULES_MCTS_STATE_PARAMETERS_HPP_
